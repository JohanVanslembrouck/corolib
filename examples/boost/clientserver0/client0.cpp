/** 
 *  @file client0.cpp
 *  @brief
 *  Using Boost ASIO to communicate between a client and an echo server.
 *  
 *  This example uses the synchronous API functions connect, write, read and close.
 *  With this API, mainflow() calls are executed sequentially, i.e. one after the other.
 *  
 *  It is possible to run mainflow() on separate threads to try to reduce the start-to-end elapsed time.
 *  This elapsed time will only be reduced if the server can handle several requests concurrently or asynchronously.
 *  This is the case with the server implementation in server1.cpp, but not with the one in server.cpp.
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

#include "print.h"

std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred)
{
    //print("completionCondition: bytes_transferred = %ld\n", bytes_transferred);
    if (!bytes_transferred)
    {
        return 1;
    }
    return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
}

void mainflow(boost::asio::io_context& ioContext, int id)
{
    print("mainflow: begin\n");

    boost::asio::ip::tcp::socket sock{ioContext};
    print("mainflow: connect to server\n");
    sock.connect(ep);
    print("mainflow: client %d got connected\n", id);

    const std::string message = "This is string " + std::to_string(id) +  " to echo\n";
    print("mainflow: sending to server: %s", message.c_str());
    print("mainflow: write\n");
    boost::asio::write(sock, boost::asio::buffer(message));

    print("mainflow: read\n");
    std::string answer;
    boost::asio::read(
            sock,
            boost::asio::dynamic_buffer(answer),
            std::bind(completionCondition, std::ref(answer), std::placeholders::_1, std::placeholders::_2));

    print("mainflow: server replied: %s", answer.c_str());
    print("mainflow: client %d got served\n", id);

    print("mainflow: close\n");
    sock.close();

    print("mainflow: end\n");
}

int main()
{
    print("main: begin\n");
    boost::asio::io_context ioContext;

    print("main: mainflow(ioContext, 0);\n");
    mainflow(ioContext, 0);
    print("main: mainflow(ioContext, 1);\n");
    mainflow(ioContext, 1);
    print("main: mainflow(ioContext, 1);\n");
    mainflow(ioContext, 2);

    std::thread thr1([&] { mainflow(ioContext, 10); });
    std::thread thr2([&] { mainflow(ioContext, 11); });
    std::thread thr3([&] { mainflow(ioContext, 12); });
    thr1.join();
    thr2.join();
    thr3.join();

    print("main: end\n");
    return 0;
}
