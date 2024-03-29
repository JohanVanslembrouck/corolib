/**
 * @file server.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 *
 * See README.md for further information.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>
#include <corolib/when_any.h>

#include <commserver.h>

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
	 * @brief one_client the interaction with one client. It performs the following steps:
	 * 1) Reads the request from the client
	 * 2) Starts an asynchronous timer of 1000 ms to simulate a long calculation
	 * 3) Starts reading a possible second request to cancel the first one.
	 * 4) Enters a loop where it waits for either 
	 *	  a) the timer to expire: one_client writes the reply to the client
	 *       one_client uses a counter to distinguish between two replies:
	 *       if the counter > 0, it sends a "FBK" reply, decrements the timer and restarts the timer
	 *		 if the counter == 0, it sens a "RES" reply and leaves the loop
	 *    b) a second request to cancel the action started after the first request
     *    c) the write action to complete
	 * 5) Closes the connection
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

		async_operation<void> sw;

		// Wait for either
		// a) the timer to expire
		// b) the action to be cancelled by the client
		// c) the write action to complete
		// whichever occurs first.

        print(PRI1, "one_client: when_any war( { &st, &sr2, &sw } ) ;\n");
        when_any war({ &st, &sr2, &sw });

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
				sw = commClient->start_writing(strout.c_str(), strout.length() + 1);

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
			case 2: // Write completed
			{
				print(PRI1, "one_client: i = %d: write completed\n", i);
			}
			break;
			default:
				print(PRI1, "one_client: i = %d: should not occur\n", i);
			} // switch (i)
		} // while (!done)

		print(PRI1, "one_client: co_await sw;\n");
		co_await sw;
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

    /**
     * @brief mainflow implements a potentially eternal loop.
     * mainflow creates a client object and then starts accepting a connection from a new client.
     * When a client connects, mainflow starts communicating with that client by calling one_client.
     * It does not await the completeion of that communication, but it immediately creates a new client object
     * and starts waiting for another client to connect.
     * @return async_task<int> with value 0
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
