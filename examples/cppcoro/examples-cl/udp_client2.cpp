/**
* @file udp_client2.cpp
* @brief
* Based upon ../examples-cc/udp_client2.cpp
* 
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

#include <iostream>
void CHECK(bool x)
{ 
    if (!x) std::cout << "CHECK: error\n"; 
}

void FAIL(const char* str)
{
    std::cout << "FAIL: " << str << "\n";
}

cppcoro_wrapper cc_wrapper;

async_task<int> client(io_service& ioSvc, std::optional<ipv4_endpoint>& serverAddress)
{
    auto socket = socket::create_udpv4(ioSvc);

    // don't need to bind(), should be implicitly bound on first send_to().

    // Send first message of 50 bytes
    {
        std::uint8_t buffer[50] = { 0 };
        // Original statement:
        // co_await socket.send_to(*serverAddress, buffer, 50);
        co_await cc_wrapper.send_to(socket, *serverAddress, buffer, 50);
    }

    // Receive ACK message
    {
        std::uint8_t buffer[1];
        // Original statement:
        // auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        auto [bytesReceived, ackAddress] = co_await cc_wrapper.recv_from(socket, buffer, 1);
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 0);
        CHECK(ackAddress == *serverAddress);
    }

    // Send second message of 128 bytes
    {
        std::uint8_t buffer[128] = { 0 };
        // Original statement:
        // co_await socket.send_to(*serverAddress, buffer, 128);
        co_await cc_wrapper.send_to(socket, *serverAddress, buffer, 128);
    }

    // Receive NACK message
    {
        std::uint8_t buffer[1];
        // Original statement:
        // auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        auto [bytesReceived, ackAddress] = co_await cc_wrapper.recv_from(socket, buffer, 1);
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 1);
        CHECK(ackAddress == *serverAddress);
    }

    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await client(ioSvc, serverAddress);
    ioSvc.stop();
    co_return 0;
}

int main()
{
    std::cout << "main: entering\n";
    io_service ioSvc;
    mainflow(ioSvc);
    ioSvc.process_events();
    std::cout << "main: leaving\n";
    return 0;
}
