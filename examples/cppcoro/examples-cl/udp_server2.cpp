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

#include "../examples-cc/check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<int> server(socket_wrapper& serverSocket)
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

async_task<void> mainflow(io_service& ioSvc)
{
    ip_endpoint serverAddress;

    auto serverSocket = socket::create_udpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    serverAddress = serverSocket.local_endpoint();
    saveServerAddress(serverAddress);

    socket_wrapper socketWrapper(serverSocket);

    co_await server(socketWrapper);

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
