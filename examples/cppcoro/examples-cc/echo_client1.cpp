/**
* @file echo_client1.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/socket_tests.cpp
* Client and server part have been placed in separate files.
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

	auto echoClient = [&]() -> task<int> {
		auto connectingSocket = socket::create_tcpv4(ioSvc);

		connectingSocket.bind(ipv4_endpoint{});

		co_await connectingSocket.connect(*serverAddress);

		auto receive = [&]() -> task<int> {
			std::uint8_t buffer[100];
			std::uint64_t totalBytesReceived = 0;
			std::size_t bytesReceived;
			do
			{
				bytesReceived = co_await connectingSocket.recv(buffer, sizeof(buffer));
				for (std::size_t i = 0; i < bytesReceived; ++i)
				{
					std::uint64_t byteIndex = totalBytesReceived + i;
					std::uint8_t expectedByte = 'a' + (byteIndex % 26);
					if (buffer[i] != expectedByte)
						printf("buffer[i] != expectedByte\n");
				}
				totalBytesReceived += bytesReceived;
			} while (bytesReceived > 0);

			if (totalBytesReceived != 1000)
				printf("totalBytesReceived = %llu != 1000\n", totalBytesReceived);

			co_return 0;
		};

		auto send = [&]() -> task<int> {
			std::uint8_t buffer[100];
			for (std::uint64_t i = 0; i < 1000; i += sizeof(buffer))
			{
				for (std::size_t j = 0; j < sizeof(buffer); ++j)
				{
					buffer[j] = 'a' + ((i + j) % 26);
				}

				std::size_t bytesSent = 0;
				do
				{
					bytesSent += co_await connectingSocket.send(
						buffer + bytesSent, sizeof(buffer) - bytesSent);
				} while (bytesSent < sizeof(buffer));
			}

			connectingSocket.close_send();

			co_return 0;
		};

		co_await when_all(send(), receive());

		co_await connectingSocket.disconnect();

		co_return 0;
	};

	(void)sync_wait(
        when_all(
		    [&]() -> task<int> {
			    auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
			    co_await echoClient();
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
