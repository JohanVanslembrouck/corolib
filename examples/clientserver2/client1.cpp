/** 
 * @file client1.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a client application.
 * This example uses 1 CommClient object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include "corolib/print.h"
#include "corolib/async_operation.h"
#include "corolib/commclient.h"
#include "corolib/async_task.h"
#include "corolib/wait_any_awaitable.h"

#include "endpoints.h"

using namespace corolib;

extern const int corolib::priority = 0x01;

boost::asio::io_context ioContext;

class ClientApp : public CommClient
{
public:
	ClientApp(
		boost::asio::io_context & ioContext,
		boost::asio::ip::tcp::endpoint ep) :
		CommClient(ioContext, ep)
	{
		print(PRI1, "ClientApp::ClientApp(...)\n");
	}

	async_task<int> acknowledgeAction()
	{
		// Prepare the ACK request
		std::string str1 = "ACK\n";

		// Writing
		print(PRI1, "acknowledgeAction: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "acknowledgeAction: co_await sw;\n");
		co_await sw;

		print(PRI1, "acknowledgeAction: co_return 0;\n");
		co_return 0;
	}

	async_task<int> cancelAction()
	{
		// Prepare the STOP request
		std::string str1 = "STOP\n";

		// Writing
		print(PRI1, "cancelAction: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "cancelAction: co_await sw;\n");
		co_await sw;

		// Start a timer of 100 ms
		// Timing
		steady_timer client_timer(ioContext);
		print(PRI1, "cancelAction: async_operation st = start_timer(100);\n");
		async_operation st = start_timer(client_timer, 100);
		print(PRI1, "cancelAction: co_await st\n");
		co_await st;

		print(PRI1, "cancelAction: co_return 0;\n");
		co_return 0;
	}
	
	async_task<int> performAction(int timeout)
	{
		// Prepare the START request
		std::string str1 = "START\n";

		// Writing
		print(PRI1, "performAction: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "performAction: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "performAction: async_operation sr = start_reading();\n");
		async_operation_t<std::string> sr = start_reading();

		// Timing
		steady_timer client_timer(ioContext);
		print(PRI1, "performAction: async_operation st = start_timer(timeout);\n");
		async_operation st = start_timer(client_timer, timeout);

		print(PRI1, "performAction: wait_all_awaitable<async_operation> war( { &sr, &st } ) ;\n");
		wait_any_awaitable<async_operation> war( { &sr, &st } );
		print(PRI1, "performAction: int i = co_await war;\n");
		int i = co_await war;

		switch (i)
		{
		case 0:	// Reply has arrived, stop the timer and print the result
		{
			print(PRI1, "performAction: i = %d: reply has arrived, stop the timer and print the result\n", i);
			
			print(PRI1, "performAction: client_timer.cancel();\n");
			client_timer.cancel();

			print(PRI1, "performAction: sr.get_result() = %s\n", sr.get_result().c_str());

			// Send acknowledgement to server
			print(PRI1, "performAction: async_task<int> ackAction = acknowledgeAction();\n");
			async_task<int> ackAction = acknowledgeAction();
			print(PRI1, "performAction: co_await ackAction;\n");
			co_await ackAction;
			print(PRI1, "performAction: after co_await ackAction;\n");
		}
		break;
		case 1: // Timer has expired, send a cancel request
		{
			print(PRI1, "performAction: i = %d: timer has expired, send a cancel request\n", i);
			
			// Canceling
			print(PRI1, "performAction: async_task<int> cnclAction = cancelAction();\n");
			async_task<int> cnclAction = cancelAction();
			print(PRI1, "performAction: co_await cnclAction;\n");
			co_await cnclAction;
			print(PRI1, "performAction: after co_await cnclAction;\n");

			// Stop reading the reply from the server (if possible)
		}
		break;
		default:
			print(PRI1, "performAction: i = %d: should not occur\n", i);
		}

		print(PRI1, "performAction: co_return 0;\n");
		co_return 0;
	}
	
	async_task<int> mainflow()
	{
		print(PRI1, "mainflow: begin\n");
		int counter = 0;

		for (int i = 0; i < 20; i++)
		{
			// Connecting
			print(PRI1, "mainflow: ------------------------------------------------------------------\n");
			print(PRI1, "mainflow: async_operation sc = start_connecting();\n");
			async_operation sc = start_connecting();
			print(PRI1, "mainflow: co_await sc;\n");
			co_await sc;
			
			if (i == 0)
			{
				// Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
				print(PRI1, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(3000));\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(3000));
			}

			// The server waits 1000 ms before sending the response.
			// Alternating, wait 2000 ms or only 200 ms.
			print(PRI1, "mainflow: async_task<int> pA = performAction(...);\n");
			async_task<int> pA = performAction((i % 2) ? 2000 : 200);
			print(PRI1, "mainflow: co_await pA;\n");
			co_await pA;
			
			// Closing
			print(PRI1, "mainflow: stop();\n");
			stop();

			print(PRI1, "mainflow: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		print(PRI1, "mainflow: co_return 0;\n");
		co_return 0;
	}
};

int main()
{
	print(PRI1, "main: ClientApp c1(ioContext, ep);\n");
	ClientApp c1(ioContext, ep);

	print(PRI1, "main: async_task<int> si = mainflow(c1);\n");
	async_task<int> si = c1.mainflow();

	print(PRI1, "main: before ioContext.run();\n");
	ioContext.run();
	print(PRI1, "main: after ioContext.run();\n");

	print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
	std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
	return 0;
}