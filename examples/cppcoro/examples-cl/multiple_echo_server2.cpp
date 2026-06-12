/**
* @file echoN_server1_client1.cpp
* @brief
* Based upon TEST_CASE("send/recv TCP/IPv4 many connections")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
*
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/on_scope_exit.hpp>
#include <cppcoro/cancellation_source.hpp>
#include <cppcoro/cancellation_token.hpp>
#include <cppcoro/async_scope.hpp>

#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include "../examples-cc/check.hpp"

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<void> handleConnection(socket s_)
{
    std::uint8_t buffer[64];
    std::size_t bytesReceived;

    socket_wrapper s(s_);
    do
    {
        bytesReceived = co_await s.recv(buffer, sizeof(buffer));
        if (bytesReceived > 0)
        {
            std::size_t bytesSent = 0;
            do
            {
                bytesSent += co_await s.send(
                    buffer + bytesSent,
                    bytesReceived - bytesSent);
            } while (bytesSent < bytesReceived);
        }
    } while (bytesReceived > 0);

    s.close_send();

    co_await s.disconnect();
}

async_task<void> echoServer(io_service& ioSvc, cppcoro::net::socket& listeningSocket, cancellation_token ct, int count)
{
    async_scope connectionScope;

    std::exception_ptr ex;
    try
    {
        //while (true) {
        for (int i = 0; i < count; ++i) {
            auto acceptingSocket_ = socket::create_tcpv4(ioSvc);
            socket_wrapper listeningSocketWr(listeningSocket);
            co_await listeningSocketWr.accept(acceptingSocket_, ct);  // blocks here in Linux
            socket_wrapper acceptingSocket(acceptingSocket_);

            connectionScope.spawn(
                handleConnection(std::move(acceptingSocket_)));
        }
    }
    catch (const cppcoro::operation_cancelled&)
    {
    }
    catch (...)
    {
        ex = std::current_exception();
    }

    co_await connectionScope.join();

    if (ex)
    {
        std::rethrow_exception(ex);
    }
}

async_task<void> mainflow(io_service& ioSvc)
{
    auto listeningSocket = socket::create_tcpv4(ioSvc);

    listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    listeningSocket.listen(20);

    cancellation_source canceller;
    int count = 20;

    async_task<void> ts = echoServer(ioSvc, listeningSocket, canceller.token(), count);
    co_await ts;
    ioSvc.stop();
    co_return;
}

async_task<void> task1(io_service& ioSvc, cppcoro::net::socket& listeningSocket, cancellation_source& canceller, int count)
{
    auto stopOnExit = on_scope_exit([&] { 
        printf("task1: on_scope_exit\n");
        ioSvc.stop();
    });

    async_task<void> ts = echoServer(ioSvc, listeningSocket, canceller.token(), count);
    co_await ts;
    co_return;
}

async_task<void> task2(io_service& ioSvc)
{
    ioSvc.process_events();
    co_return;
}

//TEST_CASE("send/recv TCP/IPv4 many connections")
async_task<void> mainflow2(io_service& ioSvc)
{
	auto listeningSocket = socket::create_tcpv4(ioSvc);

	listeningSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
	listeningSocket.listen(20);

    ip_endpoint serverAddress = listeningSocket.local_endpoint();
    saveServerAddress(serverAddress);

	cancellation_source canceller;
    int count = 20;

    async_task<void> t1 = task1(ioSvc, listeningSocket, canceller, count);
    async_task<void> t2 = task2(ioSvc);
    co_await when_all(t1, t2);
    co_return;
}

void mainflow()
{
    io_service ioSvc;
#if 0
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
#else
    async_task<void> t = mainflow2(ioSvc);
#endif
    t.wait();
}

int main()
{
    set_print_level(0x01);      // Use 0x03 to follow the flow in corolib
    mainflow();
    return 0;
}
