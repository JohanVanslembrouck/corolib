/**
* @file echo_client2.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* Lambdas have been replaced with normal functions.
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

#include "addressfile.hpp"
#include "check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

task<int> receive(socket sock)
{
	std::uint8_t buffer[100];
	std::uint64_t totalBytesReceived = 0;
	std::size_t bytesReceived;
	do
	{
		bytesReceived = co_await sock.recv(buffer, sizeof(buffer));
		for (std::size_t i = 0; i < bytesReceived; ++i)
		{
			std::uint64_t byteIndex = totalBytesReceived + i;
			std::uint8_t expectedByte = 'a' + (byteIndex % 26);
            CHECK(buffer[i] == expectedByte);
		}
		totalBytesReceived += bytesReceived;
	} while (bytesReceived > 0);

    CHECK(totalBytesReceived == 1000);
	co_return 0;
}

task<int> send(socket sock)
{
	std::uint8_t buffer[100];
    std::size_t totalBytesSent = 0;
	for (std::uint64_t i = 0; i < 1000; i += sizeof(buffer))
	{
		for (std::size_t j = 0; j < sizeof(buffer); ++j)
		{
			buffer[j] = 'a' + ((i + j) % 26);
		}

		std::size_t bytesSent = 0;
		do
		{
			bytesSent +=
				co_await sock.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
            totalBytesSent += bytesSent;
		} while (bytesSent < sizeof(buffer));
	}

    sock.close_send();

	co_return 0;
}

 task<int> echoClient(io_service& ioSvc, ipv4_endpoint& serverAddress)
 {
	socket connectingSocket = socket::create_tcpv4(ioSvc);

	connectingSocket.bind(ipv4_endpoint{});

	co_await connectingSocket.connect(serverAddress);

	co_await when_all(send(connectingSocket),
                      receive(connectingSocket));

	co_await connectingSocket.disconnect();

	co_return 0;
};

task<int> task1(io_service& ioSvc, ipv4_endpoint& serverAddress)
{
	auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
	co_await echoClient(ioSvc, serverAddress);
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

    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

	(void)sync_wait(
        when_all(
            task1(ioSvc, serverAddress),
            task2(ioSvc)
        ));
#if 0
    (void)sync_wait(
        when_all(
            [&]() -> task<int> {
                auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
                co_await echoClient(ioSvc, serverAddress);
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
