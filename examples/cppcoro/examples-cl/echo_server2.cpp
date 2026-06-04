/**
* @file echo_server2.cpp
* @brief
* Based upon ../examples-cc/echo_server2.cpp
* 
* @author Johan Vanslembrouck
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

#define USE_CPPCORO 0

async_task<int> echoServer(io_service& ioSvc, socket& listeningSocket)
{
#if USE_CPPCORO
    auto acceptingSocket = socket::create_tcpv4(ioSvc);
#else
    auto acceptingSocket_ = socket::create_tcpv4(ioSvc);
    socket_wrapper acceptingSocket(acceptingSocket_);
#endif

#if USE_CPPCORO
    co_await listeningSocket.accept(acceptingSocket);
#else
    co_await acceptingSocket.acceptOn(listeningSocket);
#endif

    std::uint8_t buffer[64];
    std::size_t bytesReceived;
    do
    {
        bytesReceived = co_await acceptingSocket.recv(buffer, sizeof(buffer));

        if (bytesReceived > 0)
        {
            std::size_t bytesSent = 0;
            do
            {
                bytesSent +=
                    co_await acceptingSocket.send(buffer + bytesSent, bytesReceived - bytesSent);
               
            } while (bytesSent < bytesReceived);
        }
    } while (bytesReceived > 0);

    acceptingSocket.close_send();

    co_await acceptingSocket.disconnect();

    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    socket listeningSocket = socket::create_tcpv4(ioSvc);

    listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0});
    listeningSocket.listen(3);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

    co_await echoServer(ioSvc, listeningSocket);

    ioSvc.stop();

    co_return;
}

void mainflow()
{
    io_service ioSvc;
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
    t.wait();
}

int main()
{
    set_print_level(0x01);      // Use 0x03 to follow the flow in corolib
    mainflow();
    return 0;
}
