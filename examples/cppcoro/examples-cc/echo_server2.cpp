/**
* @file echo_server2.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* Lambdas have been replaced with normal functions.
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/net/socket.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/io_service.hpp>
#include <cppcoro/on_scope_exit.hpp>

#include <cppcoro/task.hpp>
#include <cppcoro/sync_wait.hpp>
#include <cppcoro/when_all.hpp>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

task<int> echoServer(io_service& ioSvc, socket& listeningSocket)
{
    auto acceptingSocket = socket::create_tcpv4(ioSvc);

    co_await listeningSocket.accept(acceptingSocket);

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
                    co_await acceptingSocket.send(
                        buffer + bytesSent,
                        bytesReceived - bytesSent);
            } while (bytesSent < bytesReceived);
        }
    } while (bytesReceived > 0);

    acceptingSocket.close_send();

    co_await acceptingSocket.disconnect();

    co_return 0;
}

task<int> task1(io_service& ioSvc, socket& listeningSocket)
{
	auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
	co_await echoServer(ioSvc, listeningSocket);
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

	socket listeningSocket = socket::create_tcpv4(ioSvc);

	listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0});
	listeningSocket.listen(3);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

	(void)sync_wait(
        when_all(
            task1(ioSvc, listeningSocket),
            task2(ioSvc)
        ));
#if 0
    (void)sync_wait(
        when_all(
            [&]() -> task<int> {
                auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
                co_await echoServer(ioSvc, listeningSocket);
                co_return 0;
            }(),
            [&]() -> task<int> {
                ioSvc.process_events();
                co_return 0;
            }()
        ));
#endif
}

int main()
{
	mainflow();
	return 0;
}
