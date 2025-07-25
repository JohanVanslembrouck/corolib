/**
* @file cl_client2.cpp
* @brief
* Based upon ../examples-cc/cd_client2.cpp
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<int> client(io_service& ioSvc, std::optional<ipv4_endpoint>& serverAddress)
{
    print(PRI5, "client - entering\n");

    auto s = socket::create_tcpv4(ioSvc);
    s.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    socket_wrapper sw(s);

    // Original statement:
    // co_await s.connect(*serverAddress);
    co_await sw.connect(*serverAddress);
    print(PRI5, "client - after co_await sw.connect\n");
    // Original statement:
    // co_await s.disconnect();
    co_await sw.disconnect();
    print(PRI5, "client - co_await sw.disconnect\n");
    print(PRI5, "client - leaving\n");
    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    print(PRI5, "mainflow - entering\n");
    std::string serverAddressStr = readServerAddress();
    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await client(ioSvc, serverAddress);
    print(PRI5, "mainflow - after co_await client\n");

    ioSvc.stop();
    print(PRI5, "mainflow - leaving\n");
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
