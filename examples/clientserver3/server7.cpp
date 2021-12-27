/**
 * @file server7.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 * 
 * Because we can now co_await the lambda passed to the dispatcher,
 * the asynchronous operation that is completed by the dispatcher (in server4.cpp, server5.cpp and server6.cpp) 
 * has become obsolete.
 * This operation has been removed in server7.cpp.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include "corolib/print.h"
#include "corolib/async_task.h"
#include "corolib/commserver.h"
#include "corolib/async_operation.h"
#include "corolib/oneway_task.h"
#include "corolib/wait_any_awaitable.h"

#include "endpoints.h"

using namespace corolib;

#include "dispatcher4.h"
#include "serverrequest2.h"

class ServerApp : public CommServer
{
public:
	ServerApp(
		boost::asio::io_context& ioContext,
		unsigned short CommService) :
		CommServer(ioContext, CommService)
	{
		print(PRI1, "ServerApp::ServerApp(...)\n");
	}

	/**
	 *
	 *
	 *
	 */
	async_task<int> read_client_request(spCommCore commClient, Dispatcher& dispatcher)
	{
		print(PRI1, "read_client_request: entry\n");
		while (1)
		{
			// Reading
			print(PRI1, "read_client_request: async_operation<std::string> sr = commClient->start_reading();\n");
			async_operation<std::string> sr = commClient->start_reading();
			print(PRI1, "read_client_request: std::string str = co_await sr;\n");
			std::string str = co_await sr;
			//print(PRI1, "read_client_request: std::string str = %s\n", str.c_str());

			if (str.compare("EOF") == 0)
				break;

			print(PRI1, "read_client_request: dispatcher.dispatch(sr);\n");
			// In reality, str will contain the identification and the marshalled arguments
			async_task<int> t = dispatcher.dispatch(str);
			co_await t;
		}

		print(PRI1, "mainflow_one_client: commClient->stop();\n");
		commClient->stop();

		print(PRI1, "mainflow_one_client: co_return 0;\n");
		co_return 0;
	}
	
	/**
	 *
	 *
	 *
	 */
	oneway_task main_one_client(spCommCore commClient)
	{
		Dispatcher dispatcher;
		ServerRequest serverRequest(commClient, m_IoContext);

		async_operation<std::string> reqs[4];

		dispatcher.registerFunctor(
			"Req1",
			[&serverRequest](std::string str) -> async_task<int>
			{
				// TODO: unmarshal str into Req1
				Req1 req1;
				async_task<int> t = serverRequest.operation1(req1);
				co_await t;
				co_return 0;
			});

		dispatcher.registerFunctor(
			"Req2",
			[&serverRequest](std::string str) -> async_task<int>
			{
				// TODO: unmarshal str into Req2
				Req2 req2;
				async_task<int> t = serverRequest.operation2(req2);
				co_await t;
				co_return 0;
			});

		dispatcher.registerFunctor(
			"Req3",
			[&serverRequest](std::string str) -> async_task<int>
			{
				// TODO: unmarshal str into Req3
				Req3 req3;
				async_task<int> t = serverRequest.operation3(req3);
				co_await t;
				co_return 0;
			});

		dispatcher.registerFunctor(
			"Req4",
			[&serverRequest](std::string str) -> async_task<int>
			{
				// TODO: unmarshal str into Req4
				Req4 req4;
				async_task<int> t = serverRequest.operation4(req4);
				co_await t;
				co_return 0;
			});
		
		print(PRI1, "main_one_client: async_task<int> rcr = read_client_request(commClient, dispatcher);\n");
		async_task<int> rcr = read_client_request(commClient, dispatcher);
		print(PRI1, "main_one_client: int i = co_await rcr;\n");
		int i = co_await rcr;
	}

	/**
	 *
	 *
	 *
	 */
	async_task<int> mainflow()
	{
		int counter = 0;
		while (1)
		{
			print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", counter++);
			spCommCore commCore = std::make_shared<CommCore>(m_IoContext);

			// Accepting
			print(PRI1, "mainflow: async_operation<void> sa = start_accepting(commCore);\n");
			async_operation<void> sa = start_accepting(commCore);
			print(PRI1, "mainflow: co_await sa;\n");
			co_await sa;

			// Start communicating asynchronously with the new client.
			// Start accepting new connections immediately.
			print(PRI1, "mainflow: main_one_client(commCore);\n");
			(void)main_one_client(commCore);
		}

		print(PRI1, "mainflow: co_return 0;\n");
		co_return 0;
	}
};

std::atomic_bool stop{ false };

void runServer(
	ServerApp& server, 
	boost::asio::io_context& ioContext)
{
	print(PRI1, "runServer(...): async_task<int> si = server.mainflow();\n");
	async_task<int> si = server.mainflow();

	print(PRI1, "runServer(...): before ioContext.run();\n");
	ioContext.run();
	print(PRI1, "runServer(...): after ioContext.run();\n");
}

void handlerSignals(
	const boost::system::error_code& erc, 
	int signal)
{
	(void)erc;
    print(PRI1, "handlerSignals(...): signal %d occurred. Time to stop the application", signal);
    stop = true;
}

void asyncSignal(boost::asio::io_context& ioContext)
{
	print(PRI1, "asyncSignal(...)\n");
    boost::asio::signal_set signals{ioContext, SIGINT, SIGTERM};

    signals.async_wait(handlerSignals);
    ioContext.run();
}

int main()
{
	set_priority(0x01);

    boost::asio::io_context ioContextSignal;
    boost::asio::io_context ioContextServer;

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

	ServerApp server1{ioContextServer, port_server};

	std::thread t1{runServer, std::ref(server1), std::ref(ioContextServer)};

    t.join(); /// wait on stop signal

	print(PRI1, "main(): server1.stop()\n");
    server1.stop();
    t1.join();

	print(PRI1, "main(): return 0;\n");
	return 0;
}
