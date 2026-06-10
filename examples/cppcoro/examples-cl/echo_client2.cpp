/**
* @file echo_client2.cpp
* @brief
* Based upon ../examples-cc/echo_client2.cpp
* 
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "../examples-cc/check.hpp"

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

#define USE_CPPCORO 0

async_task<int> receive(socket_wrapper sock)
{
    std::uint8_t buffer[100];
    std::uint64_t totalBytesReceived = 0;
    std::size_t bytesReceived;
    do
    {
        bytesReceived = co_await sock.recv(buffer, sizeof(buffer));
        for (std::size_t i = 0; i < bytesReceived; ++i)
        {
            std::uint64_t byteIndex = totalBytesReceived + i;
            std::uint8_t expectedByte = 'a' + (byteIndex % 26);
            CHECK(buffer[i] == expectedByte);
        }
        totalBytesReceived += bytesReceived;
    } 
    while (bytesReceived > 0 && totalBytesReceived < 1000);

    CHECK(totalBytesReceived == 1000);
    std::cout << "Received " << totalBytesReceived << " bytes\n";

    co_return 0;
}

async_task<int> send(socket_wrapper sock)
{
    std::uint8_t buffer[100];
    std::size_t totalBytesSent = 0;
    for (std::uint64_t i = 0; i < 1000; i += sizeof(buffer))
    {
        for (std::size_t j = 0; j < sizeof(buffer); ++j)
        {
            buffer[j] = 'a' + ((i + j) % 26);
        }

        std::size_t bytesSent = 0;
        do
        {
            bytesSent +=
                co_await sock.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
            totalBytesSent += bytesSent;
        } while (bytesSent < sizeof(buffer));
    }


#if USE_CPPCORO
    // The presence of the following statement gives problems: Connection closed before bytes are received.
    sock.close_send();
#endif

    co_return 0;
}

async_task<int> echoClient(io_service& ioSvc, ipv4_endpoint& serverAddress)
{
#if USE_CPPCORO
    socket connectingSocket = socket::create_tcpv4(ioSvc);
    connectingSocket.bind(ipv4_endpoint{});
#else
    socket connectingSocket_ = socket::create_tcpv4(ioSvc);
    connectingSocket_.bind(ipv4_endpoint{});
    socket_wrapper connectingSocket(connectingSocket_);
#endif

    co_await connectingSocket.connect(serverAddress);

    async_task<int> cl = send(connectingSocket);
    async_task<int> rc = receive(connectingSocket);
    co_await when_all(cl, rc);
 
    co_await connectingSocket.disconnect();

    ioSvc.stop();

    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    std::string serverAddressStr = readServerAddress();
    std::optional<ipv4_endpoint> serverAddressAux = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);
    ipv4_endpoint serverAddress = *serverAddressAux;

    co_await echoClient(ioSvc, serverAddress);

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
    set_print_level(0x01);      // Use 0x03 to follow the flow in corolib
    mainflow();
    return 0;
}
