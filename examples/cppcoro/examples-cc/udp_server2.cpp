/**
* @file udp_server2.cpp
* @brief
* Based upon TEST_CASE("udp send_to/recv_from")
* in https://github.com/lewissbaker/cppcoro/blob/master/test/socket_tests.cpp
* Client and server part have been placed in separate files.
* Lambdas have been replaced by normal functions.
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

task<int> task1(io_service& ioSvc, cppcoro::task<int>& serverTask)
{
    auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
    (void)co_await serverTask;
    co_return 0;
}

task<int> task2(io_service& ioSvc)
{
    ioSvc.process_events();
    co_return 0;
}

void mainflow()
{
    io_service ioSvc;

    ip_endpoint serverAddress;

    task<int> serverTask;

    auto serverSocket = socket::create_udpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    serverAddress = serverSocket.local_endpoint();

    saveServerAddress(serverAddress);

    serverTask = server(std::move(serverSocket));

    (void)sync_wait(
        when_all(
            task1(ioSvc, serverTask),
            task2(ioSvc)
        ));
}

int main()
{
    std::cout << "main: entering\n";
    mainflow();
    std::cout << "main: leaving\n";
    return 0;
}
