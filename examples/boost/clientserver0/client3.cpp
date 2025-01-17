/**
 *  @file client3.cpp
 *  @brief
 *  Using Boost ASIO to communicate between a client and an echo server.
 *
 *  In contrast to client2.cpp, the main program flow (in function mainflow()) is run on a separate thread.
 *  ioContext.run() is run on yet another thread.
 *  Consequently, also the lambda functions (in the print statements called handle_connect,
 *  handle_write, handle_read) are run on the ioContext thread.
 *
 *  The mainflow thread and the ioContext thread synchronize by means of a semaphore:
 *  the mainflow thread acquires the semaphore, the ioContext thread releases it.
 *
 *  This allows defining the program flow in the mainflow() function
 *  instead of "hard-coding" it in the lambda functions as in client1.cpp.
 *
 *  This example does not use coroutines.
 *  However, the structure of the application is similar to that of an application using coroutines. 
 *  Releasing the semaphore corresponds to resuming the application.
 *  Acquiring the semaphore corresponds to a co_await call, apart from the fact
 *  that the function acquiring the semaphore will block at this point (if the semaphore
 *  has not yet been released), instead of passing control to its caller.
 *
 *  @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <future>

#include <boost/asio.hpp>

#include <thread>
#include <iostream>
#include <string>

 /// we will connect to the synchronous server of Asio10-EchoServer1
 /// we will make multiple threads that connect and get served

const boost::asio::ip::tcp::endpoint ep{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

#include "print.h"
#include "csemaphore.h"

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
        deadline_.async_wait(
            [this](const boost::system::error_code& error)
            {
                check_deadline();
            });
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
        sema.release();
    }

public:
    void start_connect()
    {
        print("client::start_connect()\n");

        tcp::resolver::results_type::iterator endpoint_iter;

        // Set a deadline for the connect operation.
        deadline_.expires_after(std::chrono::seconds(60));

        std::vector<boost::asio::ip::tcp::endpoint> eps;
        eps.push_back(ep);

        // Start the asynchronous connect operation.
        boost::asio::async_connect(
            socket_,
            eps,
            [this](const boost::system::error_code& error,
                const tcp::endpoint& result_endpoint)
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
                    sema.release();
                }
            });
    }

    void start_write()
    {
        print("client::start_write()\n");

        if (stopped_)
            return;

        const char* str = "This is the string to echo\n";
        print("client::start_write(): writing: %s", str);

        // Start an asynchronous operation to send a heartbeat message.
        boost::asio::async_write(
            socket_,
            boost::asio::buffer(str, strlen(str)),
            [this](const boost::system::error_code& error,
                std::size_t result_n)
            {
                print("client::handle_write()\n");

                if (stopped_)
                    return;

                if (!error)
                {
                    sema.release();
                }
                else
                {
                    print("client::handle_write(): error on write: %s\n", error.message().c_str());
                    stop();
                }
            });
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
            [this](const boost::system::error_code& error,
                std::size_t n)
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
                        print("client::handle_read(): received: %s\n", line.c_str());
                    }
                    sema.release();
                    stop();
                }
                else
                {
                    print("client::handle_read(): error on receive: %s\n", error.message().c_str());
                    stop();
                }
            });
    }

    void check_deadline_aux()
    {
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
    }

    void check_deadline()
    {
        print("client::check_deadline()\n");

        if (stopped_)
            return;

        check_deadline_aux();

        // Put the actor back to sleep.
        deadline_.async_wait(
            [this](const boost::system::error_code& error)
            {
                print("client::check_deadline()\n");

                if (stopped_)
                    return;

                check_deadline_aux();
            });
    }

private:
    bool stopped_ = false;
    tcp::socket socket_;
    std::string input_buffer_;
    steady_timer deadline_;
    steady_timer heartbeat_timer_;
};

void mainflow()
{
    boost::asio::io_context ioContext;

    print("client c(ioContext);\n");
    client c(ioContext);

    print("c.start();\n");
    c.start();

    print("std::thread([&ioContext] { ioContext.run(); }).detach();\n");
    std::thread([&ioContext] { ioContext.run(); }).detach();

    print("sema.acquire();\n");
    sema.acquire();

    print("c.start_write();\n");
    c.start_write();
    print("sema.acquire();\n");
    sema.acquire();

    print("c.start_read();\n");
    c.start_read();
    print("sema.acquire();\n");
    sema.acquire();

    // Waiting for stop
    sema.acquire();
}

int main()
{
    print("start\n");

    std::future<void> t1 = std::async(std::launch::async, []() { mainflow(); });
    std::future<void> t2 = std::async(std::launch::async, []() { mainflow(); });
    
    t1.get();
    t2.get();

    print("done\n");
}