/**
 * @file server.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commserver.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>
#include <corolib/wait_any_awaitable.h>

#include "endpoints.h"

using namespace corolib;

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
	 * @brief Handles the interaction with one client
	 * 1) Reads the request from the client
	 * 2) Starts an asynchronous timer to simulate a long calculation
	 * 3) Starts reading a possible second request to cancel the first one.
	 * 4) Waits for either 
	 *	  a) timer to expire: write the reply to the client
	 *    b) a second request to cancel the action started after the first request
	 * 5) close the connection
	 */
	oneway_task one_client(spCommCore commClient)
	{
		// Reading
		print(PRI1, "one_client: async_operation<std::string> sr1 = commClient->start_reading();\n");
		async_operation<std::string> sr1 = commClient->start_reading();
		print(PRI1, "one_client: std::string strIn = co_await sr1;\n");
		std::string strIn = co_await sr1;
		print(PRI1, "one_client: strIn = %s\n", strIn.c_str());

		// Start a timer to introduce a delay (to simulate a long asynchronous calculation)
		// before writing the reply to the client.
		// In reality, we can image the calculation to run on a separate thread.
		boost::asio::steady_timer client_timer(m_IoContext);
		print(PRI1, "one_client_write_reply: async_operation<void> st = commClient->start_timer(client_timer, 1000);\n");
		async_operation<void> st = commClient->start_timer(client_timer, 1000);

		// Start reading a possible second request, which (in this example)
		// is just a request to cancel the still running action started after the first request.
		print(PRI1, "one_client: async_operation<std::string> sr2 = commClient->start_reading();\n");
		async_operation<std::string> sr2 = commClient->start_reading();

		// Wait for either
		// a) the timer to expire
		// b) the action to be cancelled by the client
		// whichever occurs first.

		print(PRI1, "one_client: wait_any<async_operation_base> war( { &st, &sr2 } ) ;\n");
		wait_any<async_operation_base> war({ &st, &sr2 });

		int nrOfFeedbacks = 10;
		bool done = false;
		while (!done)
		{
			print(PRI1, "one_client: %d ---------------------------\n", nrOfFeedbacks);

			print(PRI1, "one_client: int i = co_await war;\n");
			int i = co_await war;

			switch (i)
			{
			case 0:	// timer has expired
			{
				print(PRI1, "one_client: i = %d: timer has expired\n", i);

				std::string strout;

				// Preparing output
				if (nrOfFeedbacks > 0)
				{
					strout = "FBK:params-go-here\n";
				}
				else
				{
					strout = "RES:params-go-here\n";
					done = true;
				}

				// Writing
				print(PRI1, "one_client: i = %d: async_operation<void> sw = commClient->start_writing(...);\n", i);
				async_operation<void> sw = commClient->start_writing(strout.c_str(), strout.length() + 1);
				print(PRI1, "one_client: i = %d: co_await sw;\n", i);
				co_await sw;

				if (nrOfFeedbacks > 0)
				{
					// Restart the timer of 1000 ms
					// Timing
					print(PRI1, "one_client: i = %d: st = commClient->start_timer(1000);\n", i);
					st = commClient->start_timer(client_timer, 1000);
				}
				nrOfFeedbacks--;
			}
			break;
			case 1: // We received a second request from the client (should be a stop request)
			{
				print(PRI1, "one_client: i = %d: second request received from client\n", i);
				
				print(PRI1, "one_client: i = %d: std::string strIn2 = sr2.get_result();\n", i);
				std::string strIn2 = sr2.get_result();
				print(PRI1, "one_client: i = %d: strIn2 = %s\n", i, strIn2.c_str());
				
				done = true;
				// Do not send a reply to the client
			}
			break;
			default:
				print(PRI1, "one_client: i = %d: should not occur\n", i);
			} // switch (i)
		} // while (!done)
#if 1
		// Without this delay there is an internal error on writing the string (onto a clossed connection?)
		print(PRI1, "one_client: st = commClient->start_timer(500);\n");
		st = commClient->start_timer(client_timer, 500);
		print(PRI1, "one_client: co_await st;\n");
		co_await st;
#endif
		// Closing
		print(PRI1, "one_client: clientSession->close();\n");
		commClient->stop();
		
		print(PRI1, "one_client: co_return;\n");
		co_return;
	}

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
			print(PRI1, "mainflow: (void)one_client(commCore);\n");
			(void)one_client(commCore);
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
