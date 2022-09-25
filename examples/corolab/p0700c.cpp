/** 
 *  Filename: p0700c.cpp
 *  Description:
 *        Using Boost ASIO to communicate between a client and an echo server.
 *        This example uses callback function.
 *        In the callback of operation N, operation N+1 is started.
 *
 *        This example does not use coroutines.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: boost_1_71_0\libs\asio\example\cpp11\timeouts\async_tcp_client.cpp
 *
 */

#include <boost/asio.hpp>

#include <thread>
#include <iostream>
#include <string>

/// we will connect to the synchronous server of Asio10-EchoServer1
/// we will make multiple threads that connect and get served

const boost::asio::ip::tcp::endpoint ep{boost::asio::ip::make_address("127.0.0.1"), 8242};

#include "print0.h"

//-------------------------------------------------------------

std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred)
{
    if(!bytes_transferred)
    {
        return 1;
    }
    return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
}

void getServed_sync(
        boost::asio::io_context& ioContext,
        int id)
{
    const std::string message = "request from client " + std::to_string(id) + " .\n";

    boost::asio::ip::tcp::socket sock{ioContext};
    sock.connect(ep);
    print("client %d got connected\n", id);

    boost::asio::write(sock, boost::asio::buffer(message));

    std::string answer;
    boost::asio::read(
            sock,
            boost::asio::dynamic_buffer(answer),
            std::bind(completionCondition, std::ref(answer), std::placeholders::_1, std::placeholders::_2));
    print("server replied : %s\n", answer.c_str());
    print("this is indeed echo of our message : %d\n", (message == answer));

    print("client %d got served\n", id);
    sock.close();
}

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
    }

private:
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
            start_write();
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
            start_read();
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

    print("getServed_sync(ioContext, 0);\n");
    getServed_sync(ioContext, 0);

    print("client c(ioContext);\n");
    client c(ioContext);
    print("c.start();\n");
    c.start();
    print("ioContext.run();\n");
    ioContext.run();

    print("done\n");
}
