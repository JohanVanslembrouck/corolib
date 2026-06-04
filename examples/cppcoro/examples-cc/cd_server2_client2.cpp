/**
* @file cd_server2_client2.cpp
* @brief
* Based upon TEST_CASE("TCP/IPv4 connect/disconnect")
* in https://github.com/andreasbuhr/cppcoro/blob/main/test/socket_tests.cpp
* Variant of cd_server1_client1.cpp: lambdas have been replaced with normal functions.
+ cd stands for connect-disconnect.
*
* @author Johan Vanslembrouck
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>
#include <cppcoro/on_scope_exit.hpp>

#include <cppcoro/sync_wait.hpp>
#include <cppcoro/task.hpp>
#include <cppcoro/when_all.hpp>

using namespace cppcoro;
using namespace cppcoro::net;

task<int> server(io_service& ioSvc, socket listeningSocket)
{
    auto s = socket::create_tcpv4(ioSvc);
    co_await listeningSocket.accept(s);
    co_await s.disconnect();
    co_return 0;
}

task<int> client(io_service& ioSvc, ip_endpoint& serverAddress)
{
    auto s = socket::create_tcpv4(ioSvc);
    s.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
    co_await s.connect(serverAddress);
    co_await s.disconnect();
    co_return 0;
}

// TEST_CASE("TCP/IPv4 connect/disconnect")
void mainflow()
{
    io_service ioSvc;

    ip_endpoint serverAddress;

    task<int> serverTask;
    
    {
        auto serverSocket = socket::create_tcpv4(ioSvc);
        serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 0 });
        serverSocket.listen(3);
        serverAddress = serverSocket.local_endpoint();
        serverTask = server(ioSvc, std::move(serverSocket));
    }

    task<int> clientTask = client(ioSvc, serverAddress);

    (void)sync_wait(when_all(
        [&]() -> task<int>
        {
            auto stopOnExit = on_scope_exit([&] { ioSvc.stop(); });
            (void)co_await when_all(std::move(serverTask), std::move(clientTask));
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
