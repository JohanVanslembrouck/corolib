/**
* @file udp_server2_client2.cpp
* @brief
* Based upon ../examples-cc/udp_server2_client2.cpp
*
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "cppcoro_wrapper.hpp"

#include "../examples-cc/check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

#define USE_CPPCORO 0

async_task<int> server(socket_wrapper& serverSocket)
{
    std::uint8_t buffer[100];

    auto [bytesReceived, remoteEndPoint] = co_await serverSocket.recv_from(buffer, 100);
    CHECK(bytesReceived == 50);

    auto remoteEndpointSave = remoteEndPoint;   // Save for later use
    
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
        // The previous statement does not throw because the exception is caught in the cppcoro_wrapper.
        // 
        if (bytesReceived == 0)                     // This indicates an error (exception) in the previous call.
            remoteEndPoint = remoteEndpointSave;    // Restore the original value of remoteEndPoint
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

async_task<int> client(io_service& ioSvc, ip_endpoint& serverAddress)
{
#if USE_CPPCORO
    auto socket = socket::create_udpv4(ioSvc);
#else
    auto socket_ = socket::create_udpv4(ioSvc);
    socket_wrapper socket(socket_);
#endif

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
    ip_endpoint serverAddress;

    auto serverSocket = socket::create_udpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    serverAddress = serverSocket.local_endpoint();
    socket_wrapper socketWrapper(serverSocket);

    async_task<int> ts = server(socketWrapper);
    async_task<int> tc = client(ioSvc, serverAddress);

    co_await when_all(ts, tc);

    ioSvc.stop();

    co_return;
}

void mainflow()
{
    io_service ioSvc;
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
    t.wait();
}

int main()
{
    set_print_level(0x11);      // Use 0x03 to follow the flow in corolib
    mainflow();
    return 0;
}
