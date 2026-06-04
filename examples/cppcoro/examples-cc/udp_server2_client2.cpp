/**
* @file udp_server2_client2.cpp
* @brief
* Based upon TEST_CASE("udp send_to/recv_from")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
* Variant of udp_server1_client1.cpp: lambdas have been replaced by normal functions.
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

#include "check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

task<int> server(socket serverSocket)
{
    std::uint8_t buffer[100];

    auto [bytesReceived, remoteEndPoint] = co_await serverSocket.recv_from(buffer, 100);
    CHECK(bytesReceived == 50);

    // Send an ACK response.
    {
        const std::uint8_t response[1] = { 0 };
        co_await serverSocket.send_to(remoteEndPoint, &response, 1);
    }

    // Second message received won't fit within buffer.
    try
    {
        std::tie(bytesReceived, remoteEndPoint) = co_await serverSocket.recv_from(buffer, 100);
        FAIL("Should have thrown");
    }
    catch (const std::system_error&)
    {
        std::cout << "co_await serverSocket.recv_from(buffer, 100) threw exception\n";
        // TODO: Map this situation to some kind of error_condition value.
        // The win32 ERROR_MORE_DATA error code doesn't seem to map to any of the standard std::errc values.
        //
        // CHECK(ex.code() == ???);
        //
        // Possibly also need to switch to returning a std::error_code directly rather than
        // throwing a std::system_error for this case.
    }

    // Send an NACK response.
    {
        const std::uint8_t response[1] = { 1 };
        co_await serverSocket.send_to(remoteEndPoint, response, 1);
    }

    co_return 0;
}

task<int> client(io_service& ioSvc, ip_endpoint& serverAddress)
{
    auto socket = socket::create_udpv4(ioSvc);

    // don't need to bind(), should be implicitly bound on first send_to().

    // Send first message of 50 bytes
    {
        std::uint8_t buffer[50] = { 0 };
        co_await socket.send_to(serverAddress, buffer, 50);
    }

    // Receive ACK message
    {
        std::uint8_t buffer[1];
        auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 0);
        CHECK(ackAddress == serverAddress);
    }

    // Send second message of 128 bytes
    {
        std::uint8_t buffer[128] = { 0 };
        co_await socket.send_to(serverAddress, buffer, 128);
    }

    // Receive NACK message
    {
        std::uint8_t buffer[1];
        auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 1);
        CHECK(ackAddress == serverAddress);
    }

    co_return 0;
}

// TEST_CASE("udp send_to/recv_from")
void mainflow()
{
    io_service ioSvc;

    ip_endpoint serverAddress;

    task<int> serverTask;

    {
        auto serverSocket = socket::create_udpv4(ioSvc);
        serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
        serverAddress = serverSocket.local_endpoint();
        serverTask = server(std::move(serverSocket));
    }

    (void)sync_wait(when_all(
        [&]() -> task<int>
        {
            auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
            (void)co_await when_all(std::move(serverTask),
                                    client(ioSvc, serverAddress));
            co_return 0;
        }(),
        [&]() -> task<int>
        {
            ioSvc.process_events();
            co_return 0;
        }()));
}

int main()
{
    mainflow();
    return 0;
}
