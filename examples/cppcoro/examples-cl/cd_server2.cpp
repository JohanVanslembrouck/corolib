/**
* @file cd_server2.cpp
* @brief
* Based upon ../examples-cc/cd_server2.cpp
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/print.h>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<int> server(io_service& ioSvc, socket& listeningSocket)
{
    print(PRI5, "server - entering\n");

    auto s = socket::create_tcpv4(ioSvc);
    socket_wrapper sw(s);

    // Original statement:
    // co_await listeningSocket.accept(s);
    co_await sw.accept(listeningSocket);
    print(PRI5, "server - after co_await sw.accept\n");
    // Original statement:
    // co_await s.disconnect();
    co_await sw.disconnect();
    print(PRI5, "server - co_await sw.disconnect\n");
    print(PRI5, "server - leaving\n");
    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    print(PRI1, "mainflow - entering\n");
    ip_endpoint serverAddress;
 
    auto serverSocket = socket::create_tcpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 8080 });
    serverSocket.listen(3);
    serverAddress = serverSocket.local_endpoint();

    saveServerAddress(serverAddress);

    co_await server(ioSvc, serverSocket);
    print(PRI1, "mainflow - after co_await server\n");

    ioSvc.stop();

    print(PRI1, "mainflow - leaving\n");
    co_return;
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
                                // Use 0x11 to follow the flow in GreeterClient
    print(PRI1, "main - entering\n");
    io_service ioSvc;
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
    t.wait();
    print(PRI1, "main - leaving\n");
    return 0;
}
