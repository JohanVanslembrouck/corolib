/**
* @file echo_server1_client1.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
* 
* @author lewis Baker | Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/when_all.hpp>
#include <cppcoro/sync_wait.hpp>
#include <cppcoro/on_scope_exit.hpp>
#include <cppcoro/cancellation_source.hpp>
#include <cppcoro/cancellation_token.hpp>
#include <cppcoro/async_scope.hpp>

#include "check.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

// TEST_CASE("send/recv TCP/IPv4")
void mainflow()
{
    io_service ioSvc;

    auto listeningSocket = socket::create_tcpv4(ioSvc);

    listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    listeningSocket.listen(3);

    auto echoServer = [&]() -> task<int>
    {
        auto acceptingSocket = socket::create_tcpv4(ioSvc);

        co_await listeningSocket.accept(acceptingSocket);

        std::uint8_t buffer[64];
        std::size_t bytesReceived;
        do
        {
            bytesReceived = co_await acceptingSocket.recv(buffer, sizeof(buffer));
            if (bytesReceived > 0)
            {
                std::size_t bytesSent = 0;
                do
                {
                    bytesSent += co_await acceptingSocket.send(
                        buffer + bytesSent,
                        bytesReceived - bytesSent);
                } while (bytesSent < bytesReceived);
            }
        } while (bytesReceived > 0);

        acceptingSocket.close_send();

        co_await acceptingSocket.disconnect();

        co_return 0;
    };

    auto echoClient = [&]() -> task<int>
    {
        auto connectingSocket = socket::create_tcpv4(ioSvc);

        connectingSocket.bind(ipv4_endpoint{});

        co_await connectingSocket.connect(listeningSocket.local_endpoint());

        auto receive = [](socket sock) -> task<int>
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
            } while (bytesReceived > 0);

            CHECK(totalBytesReceived == 1000);

            co_return 0;
        };

        auto send = [](socket sock) -> task<int>
        {
            std::uint8_t buffer[100];
            for (std::uint64_t i = 0; i < 1000; i += sizeof(buffer))
            {
                for (std::size_t j = 0; j < sizeof(buffer); ++j)
                {
                    buffer[j] = 'a' + ((i + j) % 26);
                }

                std::size_t bytesSent = 0;
                do
                {
                    bytesSent += co_await sock.send(buffer + bytesSent, sizeof(buffer) - bytesSent);
                } while (bytesSent < sizeof(buffer));
            }

            sock.close_send();

            co_return 0;
        };

        co_await when_all(send(connectingSocket), receive(connectingSocket));

        co_await connectingSocket.disconnect();

        co_return 0;
    };

    (void)sync_wait(when_all(
        [&]() -> task<int>
        {
            auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
            (void)co_await when_all(echoServer(),
                                    echoClient());
            co_return 0;
        }(),
        [&]() -> task<int>
        {
            ioSvc.process_events();
            co_return 0;
        }()));
}

int main()
{
    mainflow();
    return 0;
}
