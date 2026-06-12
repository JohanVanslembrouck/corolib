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

#include "../examples-cc/check.hpp"

async_task<int> client(io_service& ioSvc, ipv4_endpoint& serverAddress)
{
    auto socket_ = socket::create_udpv4(ioSvc);
    socket_wrapper socket(socket_);

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

async_task<void> mainflow(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

    co_await client(ioSvc, serverAddress);

    ioSvc.stop();

    co_return;
}

void mainloop()
{
    io_service ioSvc;
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
    t.wait();
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
    mainloop();
    return 0;
}
