/**
* @file udp_client2.cpp
* @brief
* Based upon ../examples-cc/udp_client2.cpp
* 
* @author Johan Vanslembrouck
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
    if (!x) print(PRI1, "CHECK: error\n");
}

void FAIL(const char* str)
{
    print(PRI1, "FAIL: %d\n", str);
}

async_task<int> client(io_service& ioSvc, std::optional<ipv4_endpoint>& serverAddress)
{
    print(PRI5, "client - entering\n");
    auto socket = socket::create_udpv4(ioSvc);

    socket_wrapper sw(socket);

    // don't need to bind(), should be implicitly bound on first send_to().

    // Send first message of 50 bytes
    {
        std::uint8_t buffer[50] = { 0 };
        // Original statement:
        // co_await socket.send_to(*serverAddress, buffer, 50);
        co_await sw.send_to(*serverAddress, buffer, 50);
        print(PRI5, "client - after\n");
    }

    // Receive ACK message
    {
        std::uint8_t buffer[1];
        // Original statement:
        // auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        auto [bytesReceived, ackAddress] = co_await sw.recv_from(buffer, 1);
        print(PRI5, "client - after co_await sw.recv_from\n");
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 0);
        CHECK(ackAddress == *serverAddress);
    }

    // Send second message of 128 bytes
    {
        std::uint8_t buffer[128] = { 0 };
        // Original statement:
        // co_await socket.send_to(*serverAddress, buffer, 128);
        co_await sw.send_to(*serverAddress, buffer, 128);
        print(PRI5, "client - after sw.send_to\n");
    }

    // Receive NACK message
    {
        std::uint8_t buffer[1];
        // Original statement:
        // auto [bytesReceived, ackAddress] = co_await socket.recv_from(buffer, 1);
        auto [bytesReceived, ackAddress] = co_await sw.recv_from(buffer, 1);
        print(PRI5, "client - after co_await sw.recv_from\n");
        CHECK(bytesReceived == 1);
        CHECK(buffer[0] == 1);
        CHECK(ackAddress == *serverAddress);
    }
    print(PRI5, "client - leaving\n");
    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    print(PRI1, "mainflow - entering\n");
    std::string serverAddressStr = readServerAddress();
    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await client(ioSvc, serverAddress);
    print(PRI1, "mainflow - co_await client\n");
    ioSvc.stop();
    print(PRI1, "mainflow - leaving\n");
    co_return 0;
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
                                // Use 0x11 to follow the flow in GreeterClient
    print(PRI1, "main - entering\n");
    io_service ioSvc;
    async_task<int> t = mainflow(ioSvc);
    ioSvc.process_events();
    int v = t.get_result();
    print(PRI1, "main - leaving\n");
    return 0;
}
