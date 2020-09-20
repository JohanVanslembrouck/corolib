/** 
 * @file client1a.cpp
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

#define SEND_ACK 1

using namespace corolib;

extern const int corolib::priority = 0x01;

int secondtimeout = 200;

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
		print(PRI1, "acknowledgeAction: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "acknowledgeAction: co_await sw;\n");
		co_await sw;

		print(PRI1, "acknowledgeAction: co_return 0;\n");
		co_return 0;
	}
	
	async_task<int> performAction(int timeout)
	{
		// Prepare the START request
		std::string str1 = "START\n";

		// Writing
		print(PRI1, "performAction: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "performAction: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "performAction: async_operation<std::string> sr = start_reading();\n");
		async_operation<std::string> sr = start_reading();

		// Timing
		steady_timer client_timer(ioContext);
		print(PRI1, "performAction: async_operation<void> st = start_timer(client_timer, timeout);\n");
		async_operation<void> st = start_timer(client_timer, timeout);
		
		print(PRI1, "performAction: wait_any_awaitable<async_operation_base> war( { &sr, &st } ) ;\n");
		wait_any_awaitable<async_operation_base> war({ &sr, &st });

		bool done = false;
		for (int j = 0; j < 2 && !done; j++)
		{
			print(PRI1, "performAction: int i = co_await war;\n");
			int i = co_await war;

			print(PRI1, "performAction: (%d, %d)\n\n", j, i);
			switch (i)
			{
			case 0:	// Reply has arrived, stop the timer and print the result
			{
				print(PRI1, "performAction: i = %d: reply has arrived, stop the timer and print the result\n", i);

				print(PRI1, "performAction: client_timer.cancel();\n");
				client_timer.cancel();

				print(PRI1, "performAction: sr.get_result() = %s\n", sr.get_result().c_str());
				
				if (j == 0)
				{
					print(PRI1, "performAction: i = %d: reply has arrived in time\n", i);
					done = true;
#if SEND_ACK
					// Send acknowledgement to server
					print(PRI1, "performAction: async_task<int> ackAction = acknowledgeAction();\n");
					async_task<int> ackAction = acknowledgeAction();
					print(PRI1, "performAction: co_await ackAction;\n");
					co_await ackAction;
#endif
				}
				else
				{
					print(PRI1, "performAction: i = %d, answer arrived after STOP was sent\n", i);
				}
			}
			break;
			case 1: 
			{
				if (j == 0)		// Timer has expired a first time, send a cancel request
				{
					print(PRI1, "performAction: i = %d: timer has expired a first time, send a cancel request\n", i);

					// Prepare the STOP request
					std::string str1 = "STOP\n";

					// Writing
					print(PRI1, "performAction: async_operation<void> sw2 = start_writing(...);\n");
					async_operation<void> sw2 = start_writing(str1.c_str(), str1.length() + 1);
					print(PRI1, "performAction: co_await sw2;\n");
					co_await sw2;

					// Start a timer of 500 ms
					// Timing
					print(PRI1, "performAction: st = start_timer(500);\n");
					st = start_timer(client_timer, 500);
				}
				else
				{
					print(PRI1, "performAction: i = %d: timer has expired a second time, do nothing\n", i);
				}
			}
			break;
			default:
				print(PRI1, "performAction: i = %d: should not occur\n", i);
			}
		}

		print(PRI1, "performAction: co_return 0;\n");
		co_return 0;
	}
	
	async_task<int> mainflow()
	{
		print(PRI1, "mainflow: begin\n");
		int counter = 0;

		for (int i = 0; i < 100; i++)
		{
			print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", i);

			// Connecting
			print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
			async_operation<void> sc = start_connecting();
			print(PRI1, "mainflow: co_await sc;\n");
			co_await sc;
			
			if (i == 0)
			{
				// Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
				steady_timer client_timer(ioContext);
				print(PRI1, "mainflow: co_await start_timer(3000);\n");
				co_await start_timer(client_timer, 3000);
			}

			// The server waits 1000 ms before sending the response.
			print(PRI1, "mainflow: async_task<int> pA = performAction(%d);\n", (i % 2) ? 2000 : secondtimeout);
			async_task<int> pA = performAction((i % 2) ? 2000 : secondtimeout);
			print(PRI1, "mainflow: co_await pA;\n");
			co_await pA;

			// Wait some time between iterations: timer has to be called before calling stop().
			// Timing
			steady_timer client_timer(ioContext);
			print(PRI1, "mainflow: co_await start_timer(100);\n");
			co_await start_timer(client_timer, 100);

			// Closing
			print(PRI1, "mainflow: stop();\n");
			stop();
		}

		print(PRI1, "mainflow: co_return 0;\n");
		co_return 0;
	}
};

int main(int argc, char* argv[])
{
	if (argc == 2)
		secondtimeout = atoi(argv[1]);

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
