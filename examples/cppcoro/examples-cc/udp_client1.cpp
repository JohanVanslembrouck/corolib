/**
* @file udp_client1.cpp
* @brief
* Based upon TEST_CASE("udp send_to/recv_from")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/when_all.hpp>
#include <cppcoro/sync_wait.hpp>
#include <cppcoro/on_scope_exit.hpp>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

#include <iostream>
void CHECK(bool x)
{ 
    if (!x) std::cout << "CHECK: error\n"; 
}

void FAIL(const char* str)
{
    std::cout << "FAIL: " << str << "\n";
}

void mainflow()
{
    io_service ioSvc;

    std::string serverAddressStr = readServerAddress();
    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    auto client = [&]() -> task<int>
        {
            auto socket = socket::create_udpv4(ioSvc);

            // don't need to bind(), should be implicitly bound on first send_to().

            // Send first message of 50 bytes
            {
                std::uint8_t buffer[50] = { 0 };
                co_await socket.send_to(*serverAddress, buffer, 50);
            }

            // Receive ACK message
            {
                std::uint8_t buffer[1];
                auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
                CHECK(bytesReceived == 1);
                CHECK(buffer[0] == 0);
                CHECK(ackAddress == *serverAddress);
            }

            // Send second message of 128 bytes
            {
                std::uint8_t buffer[128] = { 0 };
                co_await socket.send_to(*serverAddress, buffer, 128);
            }

            // Receive NACK message
            {
                std::uint8_t buffer[1];
                auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
                CHECK(bytesReceived == 1);
                CHECK(buffer[0] == 1);
                CHECK(ackAddress == *serverAddress);
            }

            co_return 0;
        };

    (void)sync_wait(
        when_all(
            [&]() -> task<int>
            {
                auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
                (void)co_await client();
                co_return 0;
            }(),
            [&]() -> task<int>
            {
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
