/**
* @file cl_client2.cpp
* @brief
* Based upon ../examples-cc/cd_client2.cpp
* 
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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
    cppcoro_wrapper cc_wrapper;

    auto s = socket::create_tcpv4(ioSvc);
    s.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });

    // Original statement:
    // co_await s.connect(*serverAddress);
    co_await cc_wrapper.connect(s, *serverAddress);
    // Original statement:
    // co_await s.disconnect();
    co_await cc_wrapper.disconnect(s);
    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await client(ioSvc, serverAddress);

    ioSvc.stop();
    co_return;
}

int main()
{
    std::cout << "main: entering\n";
    io_service ioSvc;
    mainflow(ioSvc);
    ioSvc.process_events();
    std::cout << "main: leaving\n";
	return 0;
}
