/**
* @file udp_server2.cpp
* @brief
* Based upon ../examples-cc/upd_server2.cpp
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
    if (!x) print(PRI1,"CHECK: error\n");
}

void FAIL(const char* str)
{
    print(PRI1, "FAIL: %s\n", str);
}

async_task<int> server(socket_wrapper& sw)
{
    print(PRI5, "server - entering\n");
    std::uint8_t buffer[100];

    // Original statement:
    // auto [bytesReceived, remoteEndPoint] = co_await serverSocket.recv_from(buffer, 100);
    auto [bytesReceived, remoteEndPoint] = co_await sw.recv_from(buffer, 100);
    print(PRI5, "server - after co_await sw.recv_from\n");

    auto remoteEndpointSave = remoteEndPoint;   // Save for later use
    CHECK(bytesReceived == 50);

    // Send an ACK response.
    {
        const std::uint8_t response[1] = { 0 };
        // Original statement:
        // co_await serverSocket.send_to(remoteEndPoint, &response, 1);
        co_await sw.send_to(remoteEndPoint, response, 1);
        print(PRI5, "server - after co_await sw.send_to\n");
    }

    // Second message received won't fit within buffer.
    try
    {
        // Original statement:
        // std::tie(bytesReceived, remoteEndPoint) = co_await serverSocket.recv_from(buffer, 100);
        std::tie(bytesReceived, remoteEndPoint) = co_await sw.recv_from(buffer, 100);
        print(PRI5, "server - after co_await sw.recv_from\n");
        // The previous statement does no throw because the exception is caught in the cppcoro_wrapper.
        // 
        if (bytesReceived == 0)     // This indicates an error (exception) in the previous call.
            remoteEndPoint = remoteEndpointSave;    // Restore the original value of remoteEndPoint
    }
    catch (const std::system_error&)
    {
        FAIL("co_await sw.recv_from(buffer, 100) threw exception");
    }
 
    // Send an NACK response.
    {
        const std::uint8_t response[1] = { 1 };
        // Original statement:
        // co_await serverSocket.send_to(remoteEndPoint, response, 1);
        co_await sw.send_to(remoteEndPoint, response, 1);
        print(PRI5, "server - after co_await sw.send_to\n");
    }
    print(PRI5, "server - leaving\n");
    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    print(PRI1, "mainflow - entering\n");
    ip_endpoint serverAddress;

    auto serverSocket = socket::create_udpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    serverAddress = serverSocket.local_endpoint();
    saveServerAddress(serverAddress);

    socket_wrapper sw(serverSocket);

    co_await server(sw);
    print(PRI1, "mainflow - after co_await server\n");
    ioSvc.stop();
    print(PRI1, "mainflow - entering\n");
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
