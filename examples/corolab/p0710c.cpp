/**
 *  Filename: p0710.cpp
 *  Description:
 *        Using Boost ASIO to communicate between a client and an echo server.
 *        This example uses callback function.
 *
 *        In contrast to p0700.cpp, ioContext.run() is run on a separate thread,
 *        consequently also the callback functions as run on this thread.
 *
 *        This allows defining the program flow in the main() function
 *        instead of "hard-coding" it in the callback functions as in p0700.cpp.
 *        The main thread and the callback thread synchronize by means of
 *        a semaphore.
 *
 *        This example does not use coroutines.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon:
 *
 */

#include <boost/asio.hpp>

#include <thread>
#include <iostream>
#include <string>

 /// we will connect to the synchronous server of Asio10-EchoServer1
 /// we will make multiple threads that connect and get served

const boost::asio::ip::tcp::endpoint ep{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

//--------------------------------------------------------------

/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */

uint64_t threadids[128];

int get_thread_number64(uint64_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

int get_thread_number32(uint32_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

uint64_t get_thread_id()
{
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t)(*ptr);
}

void print(const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    fprintf(stderr, "%02d: %s", threadid, msg);
}

//-------------------------------------------------------------

#include <mutex>
#include <condition_variable>

class CSemaphore
{
private:
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned int count_;
public:
    CSemaphore() : count_() { }

    void reset() {
        std::unique_lock<std::mutex> lock(mutex_);
        count_ = 0;
    }

    void signal() {
        print("CSemaphore::signal()\n");
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait() {
        std::unique_lock < std::mutex > lock(mutex_);
        while (!count_)
            condition_.wait(lock);
        --count_;
    }
};

CSemaphore sema;

//-------------------------------------------------------------

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <functional>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;
using std::placeholders::_1;
using std::placeholders::_2;

class client
{
public:
    client(boost::asio::io_context& io_context)
        : socket_(io_context),
        deadline_(io_context),
        heartbeat_timer_(io_context)
    {
        print("client::client()\n");
    }

    // Called by the user of the client class to initiate the connection process.
    // The endpoints will have been obtained using a tcp::resolver.
    void start()
    {
        print("client::start()\n");
        start_connect();

        // Start the deadline actor. You will note that we're not setting any
        // particular deadline here. Instead, the connect and input actors will
        // update the deadline prior to each asynchronous operation.
        deadline_.async_wait(std::bind(&client::check_deadline, this));
    }

    // This function terminates all the actors to shut down the connection. It
    // may be called by the user of the client class, or by the class itself in
    // response to graceful termination or an unrecoverable error.
    void stop()
    {
        print("client::stop()\n");
        stopped_ = true;
        boost::system::error_code ignored_error;
        socket_.close(ignored_error);
        deadline_.cancel();
        heartbeat_timer_.cancel();
        sema.signal();
    }

public:
    void start_connect()
    {
        print("client::start_connect()\n");

        tcp::resolver::results_type::iterator endpoint_iter;

        // Set a deadline for the connect operation.
        deadline_.expires_after(std::chrono::seconds(60));

        // Start the asynchronous connect operation.
        socket_.async_connect(
            ep,
            std::bind(&client::handle_connect, this, _1, endpoint_iter));
    }

    void handle_connect(const boost::system::error_code& error,
        tcp::resolver::results_type::iterator endpoint_iter)
    {
        print("client::handle_connect()\n");

        if (stopped_)
            return;

        // The async_connect() function automatically opens the socket at the start
        // of the asynchronous operation. If the socket is closed at this time then
        // the timeout handler must have run first.
        if (!socket_.is_open())
        {
            print("Connect timed out\n");

            // Try the next available endpoint.
            start_connect();
        }

        // Check if the connect operation failed before the deadline expired.
        else if (error)
        {
            print("Connect error: %s\n", error.message().c_str());

            // We need to close the socket used in the previous connection attempt
            // before starting a new one.
            socket_.close();

            // Try the next available endpoint.
            start_connect();
        }

        // Otherwise we have successfully established a connection.
        else
        {
            sema.signal();
        }
    }

    void start_write()
    {
        print("client::start_write()\n");

        if (stopped_)
            return;

        // Start an asynchronous operation to send a heartbeat message.
        boost::asio::async_write(
            socket_,
            boost::asio::buffer("This is the string to echo\n", 28),
            std::bind(&client::handle_write, this, _1));
    }

    void handle_write(const boost::system::error_code& error)
    {
        print("client::handle_write()\n");

        if (stopped_)
            return;

        if (!error)
        {
            sema.signal();
        }
        else
        {
            print("Error on write: %s\n", error.message().c_str());
            stop();
        }
    }

    void start_read()
    {
        print("client::start_read()\n");

        // Set a deadline for the read operation.
        deadline_.expires_after(std::chrono::seconds(30));

        // Start an asynchronous operation to read a newline-delimited message.
        boost::asio::async_read_until(
            socket_,
            boost::asio::dynamic_buffer(input_buffer_), '\n',
            std::bind(&client::handle_read, this, _1, _2));
    }

    void handle_read(const boost::system::error_code& error, std::size_t n)
    {
        print("client::handle_read()\n");

        if (stopped_)
            return;

        if (!error)
        {
            // Extract the newline-delimited message from the buffer.
            std::string line(input_buffer_.substr(0, n - 1));
            input_buffer_.erase(0, n);

            // Empty messages are heartbeats and so ignored.
            if (!line.empty())
            {
                print("Received: %s\n", line.c_str());
            }
            sema.signal();
            stop();
        }
        else
        {
            print("Error on receive: %s\n", error.message().c_str());
            stop();
        }
    }

    void check_deadline()
    {
        print("client::check_deadline()\n");

        if (stopped_)
            return;

        // Check whether the deadline has passed. We compare the deadline against
        // the current time since a new asynchronous operation may have moved the
        // deadline before this actor had a chance to run.
        if (deadline_.expiry() <= steady_timer::clock_type::now())
        {
            // The deadline has passed. The socket is closed so that any outstanding
            // asynchronous operations are cancelled.
            socket_.close();

            // There is no longer an active deadline. The expiry is set to the
            // maximum time point so that the actor takes no action until a new
            // deadline is set.
            deadline_.expires_at(steady_timer::time_point::max());
        }

        // Put the actor back to sleep.
        deadline_.async_wait(std::bind(&client::check_deadline, this));
    }

private:
    bool stopped_ = false;
    //tcp::resolver::results_type endpoints_;
    tcp::socket socket_;
    std::string input_buffer_;
    steady_timer deadline_;
    steady_timer heartbeat_timer_;
};

int main()
{
    boost::asio::io_context ioContext;

    print("start\n");

    print("client c(ioContext);\n");
    client c(ioContext);

    print("c.start();\n");
    c.start();

    print("std::thread([&ioContext] { ioContext.run(); }).detach();\n");
    std::thread([&ioContext] { ioContext.run(); }).detach();

    print("sema.wait();\n");
    sema.wait();

    print("c.start_write();\n");
    c.start_write();
    print("sema.wait();\n");
    sema.wait();

    print("c.start_read();\n");
    c.start_read();
    print("sema.wait();\n");
    sema.wait();

    // Waiting for stop
    sema.wait();

    print("done\n");
}

