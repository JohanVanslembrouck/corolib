/**
* @file udp_server2.cpp
* @brief
* Based upon ../examples-cc/upd_server2.cpp
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

async_task<int> server(socket& serverSocket)
{
    std::cout << "server: entering\n";
    std::uint8_t buffer[100];

    // Original statement:
    // auto [bytesReceived, remoteEndPoint] = co_await serverSocket.recv_from(buffer, 100);
    auto [bytesReceived, remoteEndPoint] = co_await cc_wrapper.recv_from(serverSocket, buffer, 100);
    CHECK(bytesReceived == 50);

    // Send an ACK response.
    {
        const std::uint8_t response[1] = { 0 };
        // Original statement:
        // co_await serverSocket.send_to(remoteEndPoint, &response, 1);
        std::cout << "server: before send_to 1\n";
        co_await cc_wrapper.send_to(serverSocket, remoteEndPoint, response, 1);
        std::cout << "server: after send_to 1\n";
    }

    // Second message received won't fit within buffer.
    try
    {
        // Original statement:
        // std::tie(bytesReceived, remoteEndPoint) = co_await serverSocket.recv_from(buffer, 100);
        std::tie(bytesReceived, remoteEndPoint) = co_await cc_wrapper.recv_from(serverSocket, buffer, 100);
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
        // Original statement:
        // co_await serverSocket.send_to(remoteEndPoint, response, 1);
        std::cout << "server: before send_to 2\n";
        co_await cc_wrapper.send_to(serverSocket, remoteEndPoint, response, 1);
        std::cout << "server: after send_to 2\n";
    }
    std::cout << "server: leaving\n";
    co_return 0;
}

async_task<int> mainflow(io_service& ioSvc)
{
    std::cout << "mainflow: entering\n";
    ip_endpoint serverAddress;

    auto serverSocket = socket::create_udpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    serverAddress = serverSocket.local_endpoint();
    saveServerAddress(serverAddress);

    co_await server(serverSocket);
    ioSvc.stop();
    std::cout << "mainflow: leaving\n";
    co_return 0;
}

int main()
{
    set_priority(0x01);

    std::cout << "main: entering\n";
    io_service ioSvc;
    mainflow(ioSvc);
    ioSvc.process_events();
    std::cout << "main: leaving\n";
    return 0;
}
