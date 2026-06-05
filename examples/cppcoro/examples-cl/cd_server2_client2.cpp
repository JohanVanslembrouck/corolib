/**
* @file cd_server2_client2.cpp
* @brief
* Based upon ../examples-cc/cd_server2_client2.cpp
*
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

#define USE_CPPCORO 0

async_task<int> server(io_service& ioSvc, socket& listeningSocket)
{
#if USE_CPPCORO
    auto s = socket::create_tcpv4(ioSvc);

    co_await listeningSocket.accept(s);
#else
    auto s_ = socket::create_tcpv4(ioSvc);
    socket_wrapper s(s_);

    socket_wrapper listeningSocketWr(listeningSocket);
    co_await listeningSocketWr.accept(s_);
#endif

    co_await s.disconnect();
    co_return 0;
}

async_task<int> client(io_service& ioSvc, ip_endpoint& serverAddress)
{
#if USE_CPPCORO
    auto s = socket::create_tcpv4(ioSvc);
    s.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
#else
    auto s_ = socket::create_tcpv4(ioSvc);
    s_.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    socket_wrapper s(s_);
#endif

    co_await s.connect(serverAddress);
    co_await s.disconnect();
    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    ip_endpoint serverAddress;

    auto serverSocket = socket::create_tcpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 8080 });
    serverSocket.listen(3);
    serverAddress = serverSocket.local_endpoint();

    async_task<int> ts = server(ioSvc, serverSocket);
    async_task<int> tc = client(ioSvc, serverAddress);
    co_await when_all(ts, tc);

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
