/**
* @file cd_server2.cpp
* @brief
* Based upon ../examples-cc/cd_server2.cpp
* 
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <cppcoro/io_service.hpp>
#include <cppcoro/net/ipv4_endpoint.hpp>
#include <cppcoro/net/socket.hpp>

#include <corolib/async_task.h>
#include <corolib/print.h>

#include "addressfile.hpp"
#include "cppcoro_wrapper.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

using namespace corolib;

async_task<int> server(io_service& ioSvc, socket& listeningSocket)
{
    cppcoro_wrapper cc_wrapper;

    auto s = socket::create_tcpv4(ioSvc);
    
    // Original statement:
    // co_await listeningSocket.accept(s);
    co_await cc_wrapper.accept(listeningSocket, s);
    // Original statement:
    // co_await s.disconnect();
    co_await cc_wrapper.disconnect(s);
    co_return 0;
}

async_task<void> mainflow(io_service& ioSvc)
{
    ip_endpoint serverAddress;
 
    auto serverSocket = socket::create_tcpv4(ioSvc);
    serverSocket.bind(ipv4_endpoint{ ipv4_address::loopback(), 8080 });
    serverSocket.listen(3);
    serverAddress = serverSocket.local_endpoint();

    saveServerAddress(serverAddress);

    co_await server(ioSvc, serverSocket);

    ioSvc.stop();
    co_return;
}

int main()
{
    std::cout << "main: entering\n";
    io_service ioSvc;
    async_task<void> t = mainflow(ioSvc);
    ioSvc.process_events();
    t.wait();
    std::cout << "main: leaving\n";
    return 0;
}
