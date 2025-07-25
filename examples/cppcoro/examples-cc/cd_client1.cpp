/**
* @file cd_client1.cpp
* @brief
* Based upon TEST_CASE("TCP/IPv4 connect/disconnect")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* cd stands for connect-disconnect.
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/on_scope_exit.hpp>

#include <cppcoro/sync_wait.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/when_all.hpp>

#include <iostream>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

void mainflow()
{
	io_service ioSvc;

    std::string serverAddressStr = readServerAddress();

    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

	auto client = [&]() -> task<int> {
		auto s = socket::create_tcpv4(ioSvc);
		s.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });

		co_await s.connect(*serverAddress);
		co_await s.disconnect();
		co_return 0;
	};

	task<int> clientTask = client();

	(void)sync_wait(
        when_all(
		    [&]() -> task<int> {
			    auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
                (void)co_await clientTask;
			    co_return 0;
		    }(),
		    [&]() -> task<int> {
			    ioSvc.process_events();
			    co_return 0;
		    }()
        ));
}

int main()
{
	mainflow();
	return 0;
}
