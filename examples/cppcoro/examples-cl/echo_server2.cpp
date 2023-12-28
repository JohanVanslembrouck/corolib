/**
* @file echo_server2.cpp
* @brief
* Based upon ../examples-cc/echo_server2.cpp
* 
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include <iostream>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

cppcoro_wrapper cc_wrapper;

async_task<int> echoServer(io_service& ioSvc, socket& listeningSocket)
{
    std::cout << "echoServer: entering\n";
    auto acceptingSocket = socket::create_tcpv4(ioSvc);

    // Original statement:
    // co_await listeningSocket.accept(acceptingSocket);
    co_await cc_wrapper.accept(listeningSocket, acceptingSocket);

    std::uint8_t buffer[64];
    std::size_t bytesReceived;
    std::size_t totalBytesReceived = 0;
    std::size_t totalBytesSent = 0;
    do
    {
        // Original statement:
        // bytesReceived = co_await acceptingSocket.recv(buffer, sizeof(buffer));
        bytesReceived = co_await cc_wrapper.recv(acceptingSocket, buffer, sizeof(buffer));
        std::cout << "echoServer: bytesReceived = " << bytesReceived << "\n";
        totalBytesReceived += bytesReceived;
        if (bytesReceived > 0)
        {
            std::size_t bytesSent = 0;
            std::size_t bytesSent1 = 0;
            do
            {
                // Original statement:
                // bytesSent +=
                //    co_await acceptingSocket.send(buffer + bytesSent, bytesReceived - bytesSent);
                bytesSent +=
                    co_await cc_wrapper.send(acceptingSocket, buffer + bytesSent, bytesReceived - bytesSent);
                totalBytesSent += bytesSent;
                std::cout << "echoServer: bytesSent = " << bytesSent << "\n";
            } while (bytesSent < bytesReceived);
        }
    } while (bytesReceived > 0);

    std::cout << "echoServer: totalBytesReceived = " << totalBytesReceived << ", totalBytesSent = " << totalBytesSent << "\n";

    // The presence of the following statement gives problems: server keeps hanging on this statement. FFS
    //acceptingSocket.close_send();

    // Original statement:
    // co_await acceptingSocket.disconnect();
    co_await cc_wrapper.disconnect(acceptingSocket);

    std::cout << "echoServer: leaving\n";
    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    socket listeningSocket = socket::create_tcpv4(ioSvc);

    listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0});
    listeningSocket.listen(3);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

    co_await echoServer(ioSvc, listeningSocket);

    ioSvc.stop();
    co_return 0;
}

int main()
{
    std::cout << "main: entering\n";
    io_service ioSvc;
    async_task<int> t = mainflow(ioSvc);
    ioSvc.process_events();
    int v = t.get_result();
    std::cout << "main: leaving\n";
    return 0;
}
