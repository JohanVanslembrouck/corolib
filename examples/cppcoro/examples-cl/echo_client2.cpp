/**
* @file echo_client2.cpp
* @brief
*
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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

cppcoro_wrapper cc_wrapper;
io_service ioSvc;

async_task<int> receive(socket& connectingSocket)
{
	std::uint8_t buffer[100];
	std::uint64_t totalBytesReceived = 0;
	std::size_t bytesReceived;
	do
	{
        // Original statement:
        // bytesReceived = co_await connectingSocket.recv(buffer, sizeof(buffer));
        bytesReceived = co_await cc_wrapper.recv(connectingSocket, buffer, sizeof(buffer));
        std::cout << "receive: bytesReceived = " <<  bytesReceived << "\n";
		for (std::size_t i = 0; i < bytesReceived; ++i)
		{
			std::uint64_t byteIndex = totalBytesReceived + i;
			std::uint8_t expectedByte = 'a' + (byteIndex % 26);
			if (buffer[i] != expectedByte)
				std::cout << "buffer[i] != expectedByte\n";
		}
		totalBytesReceived += bytesReceived;
        std::cout << "receive: totalBytesReceived = " << totalBytesReceived << std::endl;
	} 
    while (bytesReceived > 0 && totalBytesReceived < 1000);

	std::cout << "receive: totalBytesReceived = " << totalBytesReceived << std::endl;

	if (totalBytesReceived != 1000)
		std::cout << "receive: totalBytesReceived = " << totalBytesReceived << " |= 1000\n";

	co_return 0;
}

async_task<int> send(socket& connectingSocket)
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
            // Original statement:
            // bytesSent +=
            //    co_await connectingSocket.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
            bytesSent += 
                co_await cc_wrapper.send(connectingSocket, buffer + bytesSent, sizeof(buffer) - bytesSent);
            std::cout << "send: bytesSent = " << bytesSent << "\n";
            totalBytesSent += bytesSent;
		} while (bytesSent < sizeof(buffer));
	}

    std::cout << "send: totalBytesSent = " << totalBytesSent << "\n";

    // The presence of the following statement gives problems. FFS:
	// connectingSocket.close_send();

	co_return 0;
};

 async_task<int> echoClient(std::optional<ipv4_endpoint>& serverAddress)
 {
	socket connectingSocket = socket::create_tcpv4(ioSvc);
	connectingSocket.bind(ipv4_endpoint{});

    // Original statement:
    // co_await connectingSocket.connect(*serverAddress);
    co_await cc_wrapper.connect(connectingSocket, *serverAddress);

    async_task<int> cl = send(connectingSocket);
    async_task<int> rc = receive(connectingSocket);
    when_all<async_task<int>> wa({ &cl, &rc });
    co_await wa;

    // Original statement:
    // co_await connectingSocket.disconnect();
    co_await cc_wrapper.disconnect(connectingSocket);
    ioSvc.stop();

	co_return 0;
};

async_task<int> mainflow()
{
    std::string serverAddressStr = readServerAddress();

    auto serverAddress = cppcoro::net::ipv4_endpoint::from_string(serverAddressStr);

    co_await echoClient(serverAddress);
    co_return 0;
}

int main()
{
    std::cout << "main: entering\n";
	mainflow();
    ioSvc.process_events();
    std::cout << "main: leaving\n";
	return 0;
}
