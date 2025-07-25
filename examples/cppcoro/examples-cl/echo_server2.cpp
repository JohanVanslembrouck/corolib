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

async_task<int> echoServer(io_service& ioSvc, socket& listeningSocket)
{
    print(PRI5, "echoServer - entering\n");
    auto acceptingSocket = socket::create_tcpv4(ioSvc);

    socket_wrapper sw(acceptingSocket);

    // Original statement:
    // co_await listeningSocket.accept(acceptingSocket);
    co_await sw.accept(listeningSocket);
    print(PRI5, "echoServer - after co_await cc_wrapper.accept\n");

    std::uint8_t buffer[64];
    std::size_t bytesReceived;
    std::size_t totalBytesReceived = 0;
    std::size_t totalBytesSent = 0;
    do
    {
        // Original statement:
        // bytesReceived = co_await acceptingSocket.recv(buffer, sizeof(buffer));
        bytesReceived = co_await sw.recv(buffer, sizeof(buffer));
        print(PRI5, "echoServer - after co_await sw.recv\n");
        print(PRI1, "echoServer - bytesReceived = %d\n", bytesReceived);
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
                    co_await sw.send(buffer + bytesSent, bytesReceived - bytesSent);
                totalBytesSent += bytesSent;
                print(PRI5, "echoServer - after co_await sw.send\n");
                print(PRI1, "echoServer - bytesSent = %d\n", bytesSent);
            } while (bytesSent < bytesReceived);
        }
    } while (bytesReceived > 0);

    print(PRI1, "echoServer: totalBytesReceived = %d, totalBytesSent = %d\n", totalBytesReceived, totalBytesSent);

    // The presence of the following statement gives problems: server keeps hanging on this statement. FFS
    //acceptingSocket.close_send();

    // Original statement:
    // co_await acceptingSocket.disconnect();
    co_await sw.disconnect();
    print(PRI5, "echoServer - after co_await sz.disconnect\n");

    print(PRI5, "echoServer - leaving\n");
    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    print(PRI1, "mainflow - entering\n");
    socket listeningSocket = socket::create_tcpv4(ioSvc);

    listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0});
    listeningSocket.listen(3);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

    co_await echoServer(ioSvc, listeningSocket);
    print(PRI1, "mainflow - after co_await echoServer\n");

    ioSvc.stop();
    print(PRI1, "mainflow - entering\n");
    co_return 0;
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
                                // Use 0x11 to follow the flow in GreeterClient
    print(PRI1, "main - entering\n");
    io_service ioSvc;
    async_task<int> t = mainflow(ioSvc);
    ioSvc.process_events();
    int v = t.get_result();
    print(PRI1, "main - leaving\n");
    return 0;
}
