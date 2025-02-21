/**
 *  @file client2.cpp
 *  @brief
 *  Using Boost ASIO to communicate between a client and an echo server.
 *
 *  This example is based upon studies/corolab/p0710c.c.
 *  This example uses lambdas instead of callback functions as in p0710c.c.
 *
 *  In contrast to client1.cpp, ioContext.run() is run on a separate thread.
 *  Consequently, also the lambda functions (in the print statements called handle_connect,
 *  handle_write, handle_read) are run on the ioContext thread.
 *
 *  The main thread and the ioContext thread synchronize by means of a semaphore:
 *  the main thread acquires the semaphore, the ioContext thread releases it.
 *
 *  This allows defining the program flow in the main() function
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

#include <thread>
#include <iostream>
#include <string>

#include <boost/asio.hpp>

const boost::asio::ip::tcp::endpoint ep{ boost::asio::ip::make_address("127.0.0.1"), 8242 };

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <functional>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

#include "print.h"
#include "csemaphore.h"

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

    void start_deadline_actor()
    {
        print("client::start_deadline_actor()\n");

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
        set_result_and_release();
    }

    void get_result()
    {
        print("client::get_result()\n");
        m_sema.acquire();
    }

    void set_result_and_release()
    {
        print("client::set_result_and_release()\n");
        m_sema.release();
    }

    // Called by the user of the client class to initiate the connection process.
    // The endpoints will have been obtained using a tcp::resolver.
    void start_connecting()
    {
        print("client::start_connecting()\n");

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
                    start_connecting();
                }

                // Check if the connect operation failed before the deadline expired.
                else if (error)
                {
                    print("Connect error: %s\n", error.message().c_str());

                    // We need to close the socket used in the previous connection attempt
                    // before starting a new one.
                    socket_.close();

                    // Try the next available endpoint.
                    start_connecting();
                }

                // Otherwise we have successfully established a connection.
                else
                {
                    set_result_and_release();
                }
            });
    }

    void start_writing(std::string message)
    {
        print("client::start_writing()\n");

        if (stopped_)
            return;

        const char* str = message.c_str();
        print("client::start_writing(): writing: %s", str);

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
                    set_result_and_release();
                }
                else
                {
                    print("client::handle_write(): error on write: %s\n", error.message().c_str());
                    stop();
                }
            });
    }

    void start_reading()
    {
        print("client::start_reading()\n");

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
                    set_result_and_release();
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
    CSemaphore m_sema;
    bool stopped_ = false;
    tcp::socket socket_;
    std::string input_buffer_;
    steady_timer deadline_;
    steady_timer heartbeat_timer_;
};

void mainflow(int id)
{
    print("mainflow: begin\n");
    boost::asio::io_context ioContext;

    print("mainflow: client cl(ioContext);\n");
    client cl(ioContext);

    print("mainflow: c.start_deadline_actor();\n");
    cl.start_deadline_actor();

    print("mainflow: std::thread thr1([&ioContext] { ioContext.run(); })\n");
    std::thread thr1([&ioContext] { ioContext.run(); });

    print("mainflow: c.start_connecting();\n");
    cl.start_connecting();
    print("mainflow: cl.get_result();   // 1\n");
    cl.get_result();    // 1

    print("mainflow: cl.start_writing(...);\n");
    cl.start_writing("This is the string " + std::to_string(id) + " to echo\n");
    print("mainflow: cl.get_result();   // 2\n");
    cl.get_result();    // 2

    print("mainflow: cl.start_reading();\n");
    cl.start_reading();
    print("mainflow: cl.get_result();   // 3\n");
    cl.get_result();    // 3

    // Waiting for stop
    print("mainflow: cl.get_result();   // 4\n");
    cl.get_result();    // 4

    print("mainflow: thr1.join();\n");
    thr1.join();
    print("mainflow: end\n");
}

int main()
{
    print("main: begin\n");

    print("\n"); print("main: mainflow(0);\n");
    mainflow(0);
    print("\n"); print("main: mainflow(1);\n");
    mainflow(1);
    print("\n"); print("main: mainflow(2);\n");
    mainflow(2);

    print("main: end\n");
    return 0;
}
