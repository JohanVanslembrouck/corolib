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
#include <cppcoro/on_scope_exit.hpp>
#include <cppcoro/cancellation_source.hpp>
#include <cppcoro/cancellation_token.hpp>
#include <cppcoro/async_scope.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "../examples-cc/check.hpp"

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

#define USE_CPPCORO 0

async_task<void> receive(socket_wrapper sock)
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

async_task<void> send(socket_wrapper sock)
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

async_task<void> echoClient(io_service& ioSvc, ipv4_endpoint& serverAddress)
{
#if USE_CPPCORO
    auto connectingSocket = socket::create_tcpv4(ioSvc);

    connectingSocket.bind(ipv4_endpoint{});
#else
    auto connectingSocket_ = socket::create_tcpv4(ioSvc);

    connectingSocket_.bind(ipv4_endpoint{});
    socket_wrapper connectingSocket(connectingSocket_);
#endif

    co_await connectingSocket.connect(serverAddress);

    async_task<void> cl = send(connectingSocket);
    async_task<void> rc = receive(connectingSocket);
    co_await when_all(cl, rc);

    co_await connectingSocket.disconnect();
};

async_task<void> manyEchoClients(io_service& ioSvc, ipv4_endpoint& serverAddress, cancellation_source& canceller, int count)
{
    auto shutdownServerOnExit = on_scope_exit([&]
    {
        printf("manyEchoClients: on_scope_exit\n");
        canceller.request_cancellation();
    });

    std::vector<async_task<void>> clientTasks;
    clientTasks.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        clientTasks.emplace_back(echoClient(ioSvc, serverAddress));
    }
    
    co_await when_all(clientTasks);
};

async_task<void> mainflow(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

    cancellation_source canceller;
    int count = 20;

    async_task<void> tc = manyEchoClients(ioSvc, serverAddress, canceller, count);
    co_await tc;
    ioSvc.stop();
    co_return;
}

async_task<void> task1(io_service& ioSvc, ipv4_endpoint& serverAddress, cancellation_source& canceller, int count)
{
    auto stopOnExit = on_scope_exit([&] { 
        printf("task1: on_scope_exit\n");
        ioSvc.stop();
    });

    async_task<void> tc = manyEchoClients(ioSvc, serverAddress, canceller, count);
    co_await tc;
    co_return;
}

async_task<void> task2(io_service& ioSvc)
{
    ioSvc.process_events();
    co_return;
}

//TEST_CASE("send/recv TCP/IPv4 many connections")
async_task<void> mainflow2(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

	cancellation_source canceller;
    int count = 20;

    async_task<void> t1 = task1(ioSvc, serverAddress, canceller, count);
    async_task<void> t2 = task2(ioSvc);
    co_await when_all(t1, t2);
    co_return;
}

void mainflow()
{
    io_service ioSvc;
#if 0
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
#else
    async_task<void> t = mainflow2(ioSvc);
#endif
    t.wait();
}

int main()
{
    set_print_level(0x01);      // Use 0x03 to follow the flow in corolib
    mainflow();
    return 0;
}
