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

#include <iostream>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<int> receive(socket_wrapper& sw)
{
    print(PRI5, "receive - entering\n");
    std::uint8_t buffer[100];
    std::uint64_t totalBytesReceived = 0;
    std::size_t bytesReceived;
    do
    {
        // Original statement:
        // bytesReceived = co_await connectingSocket.recv(buffer, sizeof(buffer));
        bytesReceived = co_await sw.recv(buffer, sizeof(buffer));
        print(PRI5, "receive - after co_await sw.recv\n");
        print(PRI1, "receive - bytesReceived = %d\n", bytesReceived);
        for (std::size_t i = 0; i < bytesReceived; ++i)
        {
            std::uint64_t byteIndex = totalBytesReceived + i;
            std::uint8_t expectedByte = 'a' + (byteIndex % 26);
            if (buffer[i] != expectedByte)
                print(PRI1, "buffer[i] != expectedByte\n");
        }
        totalBytesReceived += bytesReceived;
        print(PRI1, "receive - totalBytesReceived = %d\n", totalBytesReceived);
    } 
    while (bytesReceived > 0 && totalBytesReceived < 1000);

    print(PRI1, "receive - totalBytesReceived = %d\n", totalBytesReceived);

    if (totalBytesReceived != 1000)
        print(PRI1, "receive - totalBytesReceived = %d |= 1000\n", totalBytesReceived);

    print(PRI5, "receive - leaving\n");
    co_return 0;
}

async_task<int> send(socket_wrapper& sw)
{
    print(PRI5, "send - entering\n");
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
            // Original statement:
            // bytesSent +=
            //    co_await connectingSocket.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
            bytesSent += 
                co_await sw.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
            print(PRI5, "send - after co_await sw.send\n");
            print(PRI1, "send - bytesSent = %d\n", bytesSent);
            totalBytesSent += bytesSent;
        } while (bytesSent < sizeof(buffer));
    }

    print(PRI1, "send: totalBytesSent = %d\n", totalBytesSent);

    // The presence of the following statement gives problems: Connection closed before bytes are received.
    //connectingSocket.close_send();

    print(PRI5, "send - leaving\n");
    co_return 0;
};

 async_task<int> echoClient(io_service& ioSvc, std::optional<ipv4_endpoint>& serverAddress)
 {
    print(PRI5, "echoClient - entering\n");
    socket connectingSocket = socket::create_tcpv4(ioSvc);
    connectingSocket.bind(ipv4_endpoint{});

    socket_wrapper sw(connectingSocket);

    // Original statement:
    // co_await connectingSocket.connect(*serverAddress);
    co_await sw.connect(*serverAddress);
    print(PRI5, "echoClient - after co_await sw.connect\n");

    async_task<int> cl = send(sw);
    async_task<int> rc = receive(sw);
    when_all wa({ &cl, &rc });
    co_await wa;
    print(PRI5, "echoClient - after co_await wa\n");

    // Original statement:
    // co_await connectingSocket.disconnect();
    co_await sw.disconnect();
    print(PRI5, "echoClient - after co_await sw.disconnect\n");
    ioSvc.stop();

    print(PRI5, "echoClient - leaving\n");
    co_return 0;
};

async_task<int> mainflow(io_service& ioSvc)
{
    print(PRI1, "mainflow - entering\n");
    std::string serverAddressStr = readServerAddress();

    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await echoClient(ioSvc, serverAddress);
    print(PRI1, "mainflow - after co_await echoClient\n");
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
