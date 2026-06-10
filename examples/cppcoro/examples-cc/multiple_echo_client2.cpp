/**
* @file echoN_server1_client1.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4 many connections")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
*
* @author Johan Vanslembrouck
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
#include "check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

task<> receive(socket sock)
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
    std::cout << "Received " << totalBytesReceived << " bytes\n";
}

task<> send(socket sock)
{
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
            bytesSent += co_await sock.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
        } while (bytesSent < sizeof(buffer));
    }

    sock.close_send();
}

task<> echoClient(io_service& ioSvc, ipv4_endpoint& serverAddress)
{
    auto connectingSocket = socket::create_tcpv4(ioSvc);

    connectingSocket.bind(ipv4_endpoint{});

    co_await connectingSocket.connect(serverAddress);

    co_await when_all(send(connectingSocket), receive(connectingSocket));

    co_await connectingSocket.disconnect();
}

task<void> manyEchoClients(io_service& ioSvc, ipv4_endpoint& serverAddress, cancellation_source& canceller, int count)
{
    auto shutdownServerOnExit = on_scope_exit([&]
    {
        canceller.request_cancellation();
    });

    std::vector<task<>> clientTasks;
    clientTasks.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        clientTasks.emplace_back(echoClient(ioSvc, serverAddress));
    }

    co_await when_all(std::move(clientTasks));
}

task<int> task1(io_service& ioSvc, ipv4_endpoint& serverAddress, cancellation_source& canceller, int count)
{
    auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
    co_await manyEchoClients(ioSvc, serverAddress, canceller, 20);
    co_return 0;
}

task<int> task2(io_service& ioSvc)
{
    ioSvc.process_events();
    co_return 0;
}

//TEST_CASE("send/recv TCP/IPv4 many connections")
void mainflow()
{
	io_service ioSvc;

    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

	cancellation_source canceller;

	(void)sync_wait(when_all(
		[&]() -> task<>
		{
			auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
            co_await manyEchoClients(ioSvc, serverAddress, canceller, 20);
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
