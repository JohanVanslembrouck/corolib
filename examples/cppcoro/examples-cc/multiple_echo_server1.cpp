/**
* @file echoN_server1_client1.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4 many connections")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
*
* @author Lewis Baker | Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/when_all.hpp>
#include <cppcoro/sync_wait.hpp>
#include <cppcoro/on_scope_exit.hpp>
#include <cppcoro/cancellation_source.hpp>
#include <cppcoro/cancellation_token.hpp>
#include <cppcoro/async_scope.hpp>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

//TEST_CASE("send/recv TCP/IPv4 many connections")
void mainflow()
{
	io_service ioSvc;

	auto listeningSocket = socket::create_tcpv4(ioSvc);

	listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
	listeningSocket.listen(20);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

	cancellation_source canceller;

	auto handleConnection = [](socket s) -> task<void>
	{
		std::uint8_t buffer[64];
		std::size_t bytesReceived;
		do
		{
			bytesReceived = co_await s.recv(buffer, sizeof(buffer));
			if (bytesReceived > 0)
			{
				std::size_t bytesSent = 0;
				do
				{
					bytesSent += co_await s.send(
						buffer + bytesSent,
						bytesReceived - bytesSent);
				} while (bytesSent < bytesReceived);
			}
		} while (bytesReceived > 0);

		s.close_send();

		co_await s.disconnect();
	};

	auto echoServer = [&](cancellation_token ct) -> task<>
	{
		async_scope connectionScope;

		std::exception_ptr ex;
		try
		{
			//while (true) {
            for (int i = 0; i < 20; ++i) {
				auto acceptingSocket = socket::create_tcpv4(ioSvc);
				co_await listeningSocket.accept(acceptingSocket, ct);
				connectionScope.spawn(
					handleConnection(std::move(acceptingSocket)));
			}
		}
		catch (const cppcoro::operation_cancelled&)
		{
		}
		catch (...)
		{
			ex = std::current_exception();
		}

		co_await connectionScope.join();

		if (ex)
		{
			std::rethrow_exception(ex);
		}
	};

	(void)sync_wait(when_all(
		[&]() -> task<>
		{
			auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
            co_await echoServer(canceller.token());
		}(),
		[&]() -> task<>
		{
			ioSvc.process_events();
			co_return;
		}()));
}

int main()
{
    mainflow();
    return 0;
}
