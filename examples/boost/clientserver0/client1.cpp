/** 
 *  @file client1.cpp
 *  @brief
 *  Using Boost ASIO to communicate between a client and an echo server.
 *
 *  This example is based upon studies/corolab/p0700c.c.
 *  This example mostly uses lambdas instead of callback functions as in p0700c.c.
 *  In the lambda of operation N, operation N+1 is started.
 *       
 *  This example does not use coroutines.
 *
 *  @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>
#include <iostream>
#include <string>

#include <boost/asio.hpp>

const boost::asio::ip::tcp::endpoint ep{boost::asio::ip::make_address("127.0.0.1"), 8242};

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

#include "print.h"

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
    // In this implementation, start() has to pass the message-to-echo to start_connecting()
    // and start_connecting() had to pass it to start_writing(), which uses it to write to the server.
    void start(std::string message)
    {
        print("client::start(...)\n");
        start_connecting(message);

        // Start the deadline actor. You will note that we're not setting any
        // particular deadline here. Instead, the connect and input actors will
        // update the deadline prior to each asynchronous operation.
        deadline_.async_wait(
            [this](const boost::system::error_code& error)
            {
                check_deadline();
            });
    }

private:
    void start_deadline_actor()
    {
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
    }

    void start_connecting(std::string message)
    {
        print("client::start_connecting(...)\n");

        tcp::resolver::results_type::iterator endpoint_iter;

        // Set a deadline for the connect operation.
        deadline_.expires_after(std::chrono::seconds(60));

        std::vector<boost::asio::ip::tcp::endpoint> eps;
        eps.push_back(ep);

        // Start the asynchronous connect operation.
        boost::asio::async_connect(
            socket_,
            eps,
            [this, message](const boost::system::error_code& error,
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
                    start_connecting(message);
                }

                // Check if the connect operation failed before the deadline expired.
                else if (error)
                {
                    print("Connect error: %s\n", error.message().c_str());

                    // We need to close the socket used in the previous connection attempt
                    // before starting a new one.
                    socket_.close();

                    // Try the next available endpoint.
                    start_connecting(message);
                }

                // Otherwise we have successfully established a connection.
                else
                {
                    start_writing(message);
                }
            });
    }

    void start_writing(std::string message)
    {
        print("client::start_writing(...)\n");

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
                    start_reading();
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

int main()
{
    print("main: begin\n");
    boost::asio::io_context ioContext;
    
    print("main: client cl1(ioContext);\n");
    client cl1(ioContext);
    print("main: client cl2(ioContext);\n");
    client cl2(ioContext);
    print("main: client cl2(ioContext);\n");
    client cl3(ioContext);

    print("main: cl1.start();\n");
    cl1.start("This is string 0 to echo\n");
    print("main: cl2.start();\n");
    cl2.start("This is string 1 to echo\n");
    print("main: cl3.start();\n");
    cl3.start("This is string 2 to echo\n");

    print("main: ioContext.run();\n");
    ioContext.run();

    print("main: end\n");
    return 0;
}
