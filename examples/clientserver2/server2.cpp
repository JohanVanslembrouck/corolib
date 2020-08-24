/**
 * @file server.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
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

extern const int corolib::priority = 0x01;

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
	 * @brief one_client_write_reply writes the reply to the client after having waited some time.
	 * @param commClient identification of the client to communicate with
	 * @param cancelAction allows the calling coroutine to indicate to this coroutine
	 * that it has to stop the timer and not send the reply
	 * @param completeAction allows this coroutine to indicate to the calling coroutine
	 * that it is about to send the reply to the client
	 * @note one_client_write_reply cannot return an async_operation
	 * because async_operation does not has an associated promise_type.
	 * Therefore, completeAction has been introduced because async_operation can be used
	 * as argument.
	 */
	async_task<int> one_client_write_reply(spCommCore commClient, 
		async_operation& cancelAction, async_operation& completeAction)
	{
		// Start a timer to introduce a delay (to simulate a long asynchronous calculation)
		// before writing the reply to the client.
		// In reality, we can image the calculation to run on a separate thread.
		boost::asio::steady_timer client_timer(m_IoContext);
		print(PRI1, "one_client_write_reply: async_operation st = commClient->start_timer(client_timer, 1000);\n");
		async_operation st = commClient->start_timer(client_timer, 1000);

		// Wait for either
		// a) the timer to expire
		// b) the action to be canceled by the client,
		// whichever occurs first.
		print(PRI1, "one_client_write_reply: wait_any_awaitable<async_operation> war( { &st, &cancelAction } ) ;\n");
		wait_any_awaitable<async_operation> war( { &st, &cancelAction} );
		
		print(PRI1, "one_client_write_reply: int i = co_await war;\n");
		int i = co_await war;
		
		// Look which one has completed
		switch (i)
		{
		case 0:	// Timer has expired: write the result to the client
		{
			print(PRI1, "one_client_write_reply: i = %d: timer has expired\n", i);
			
			// Preparing output
			std::string strout = "ANSWER\n";

			// Signal the completion of the action to one_client.
			// one_client co_awaits the completion of completeAction.
			print(PRI1, "one_client_write_reply: completeAction.completed();\n");
			completeAction.completed();

			// Writing
			print(PRI1, "one_client_write_reply: async_operation sw = commClient->start_writing(...);\n");
			async_operation sw = commClient->start_writing(strout.c_str(), strout.length() + 1);
			print(PRI1, "one_client_write_reply: co_await sw;\n");
			co_await sw;
		}
		break;
		case 1: // The client has stopped the action: stop the timer
			print(PRI1, "one_client_write_reply: i = %d: the client stopped the action\n", i);
			
			print(PRI1, "one_client_write_reply: client_timer.cancel();\n");
			client_timer.cancel();
		break;
		default:
			print(PRI1, "one_client_write_reply: i = %d: should not occur\n", i);
		}

		print(PRI1, "one_client_write_reply: co_return\n");
		co_return 0;
	}
	
	/**
	 * @brief Handles the interaction with one client
	 * 1) Reads the request from the client
	 * 2) Calls one_client_write_reply that will send the reply 
	 *    to the client after a delay.
	 * 3) Starts reading a possible second request to cancel the first one.
	 * 4) Waits for either 
	 *	  a) one_client_write_reply to complete its writing
	 *    b) a second request to cancel the action started after the first request
	 */
	oneway_task one_client(spCommCore commClient)
	{
		async_operation cancelAction;
		async_operation completeAction;
		
		// Reading
		print(PRI1, "one_client: async_operation sr1 = commClient->start_reading();\n");
		async_operation_t<std::string> sr1 = commClient->start_reading();
		print(PRI1, "one_client: std::string strIn = co_await sr1;\n");
		std::string strIn = co_await sr1;
		print(PRI1, "one_client: strIn = %s\n", strIn.c_str());

		// Call one_client_write_reply to send the reply to the client after some delay.
		// During this delay, the client may cancel the action.
		print(PRI1, "one_client: async_operation sw = one_client_write_reply(commClient, cancelAction, completeAction);\n");
		async_task<int> ocwr = one_client_write_reply(commClient, cancelAction, completeAction);
		
		// Start reading a possible second request, which (in this example)
		// is just a request to cancel the still running action started after the first request.
		print(PRI1, "one_client: async_operation sr2 = commClient->start_reading();\n");
		async_operation_t<std::string> sr2 = commClient->start_reading();
		
		// Wait for either
		// a) writing of the reply to the client to be started
		// b) the action to be cancelled by the client,
		// whichever occurs first.
		print(PRI1, "one_client: wait_any_awaitable<async_operation> war( { &completeAction, &sr2 } ) ;\n");
		wait_any_awaitable<async_operation> war( { &completeAction, &sr2} );
		print(PRI1, "one_client: int i = co_await war;\n");
		int i = co_await war;

		// Look which one has completed.
		switch (i)
		{
		case 0:	// one_client_write_reply has written the response to the client
		{
			print(PRI1, "one_client: i = %d: action completed by one_client_write_reply\n", i);

			// Reading ACK
			print(PRI1, "one_client: std::string strIn2 = co_await sr2;\n");
			std::string strIn2 = co_await sr2;
			print(PRI1, "one_client: strIn2 = %s\n", strIn2.c_str());
		}
		break;
		case 1: // We received a second request from the client (should be a stop request)
		{
			print(PRI1, "one_client: i = %d: second request received from client\n", i);
			
			print(PRI1, "one_client: std::string strIn2 = sr2.get_result();\n");
			std::string strIn2 = sr2.get_result();
			print(PRI1, "one_client: strIn2 = %s\n", strIn2.c_str());
			
			// Stop the current action in one_client_write_reply
			print(PRI1, "one_client: cancelAction.completed();\n");
			cancelAction.completed();
		}
		break;
		default:
			print(PRI1, "one_client: i = %d: should not occur\n", i);
		}
		
		// Await the completion of one_client_write_reply.
		print(PRI1, "one_client: co_await ocwr;\n");
		co_await ocwr;

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
			print(PRI3, "mainflow: %d -------------------------------------------------\n", counter++);
			spCommCore commCore = std::make_shared<CommCore>(m_IoContext);

			// Accepting
			print(PRI1, "mainflow: async_operation sa = start_accepting(commCore);\n");
			async_operation sa = start_accepting(commCore);
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
