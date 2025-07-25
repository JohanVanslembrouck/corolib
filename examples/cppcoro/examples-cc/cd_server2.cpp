/**
* @file cd_server2.cpp
* @brief
* Based upon TEST_CASE("TCP/IPv4 connect/disconnect")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* cd stands for connect-disconnect.
* Lambdas have been replaced by normal functions.
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

task<int> server(io_service& ioSvc, socket& listeningSocket)
{
    auto s = socket::create_tcpv4(ioSvc);
    co_await listeningSocket.accept(s);
    co_await s.disconnect();
    co_return 0;
}

task<int> task1(io_service& ioSvc, task<int>& serverTask)
{
    auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
    (void)co_await serverTask;
    co_return 0;
}

task<int> task2(io_service& ioSvc)
{
    ioSvc.process_events();
    co_return 0;
}

void mainflow()
{
	io_service ioSvc;

	ip_endpoint serverAddress;

	task<int> serverTask;

	auto serverSocket = socket::create_tcpv4(ioSvc);
	serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 8080 });
	serverSocket.listen(3);
	serverAddress = serverSocket.local_endpoint();

    saveServerAddress(serverAddress);

	serverTask = server(ioSvc, serverSocket);

	(void)sync_wait(
        when_all(
            task1(ioSvc, serverTask),
            task2(ioSvc)
	    ));
}

int main()
{
    std::cout << "main: entering\n";
	mainflow();
    std::cout << "main: leaving\n";
	return 0;
}
