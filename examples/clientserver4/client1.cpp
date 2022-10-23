/** 
 * @file client1.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a client application.
 * This example uses 1 CommClient object.
 *
 * See README.md for further information.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/commclient.h>
#include <corolib/async_task.h>
#include <corolib/when_any.h>

#include "endpoints.h"

using namespace corolib;

int secondtimeout = 5000;

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

    /**
     * @brief getHeader retrieves the header identifying the operation from a string.
     * The header is the substring preceding the first : in the string.
     * @param str is the string to retrieve the header for,
     * @return the header found in the string or the empty string if not present
     */
	std::string getHeader(std::string str)
	{
		while (str.size())
		{
			int index = str.find(':');
			if (index != std::string::npos)
			{
				return (str.substr(0, index));
			}
		}
		return "";
	}

    /**
     * @brief performAction first writes "GOAL\n" onto the connection to the server.
     * It then start reading the response and it also starts a timer.
     * Depending on the timer length, the timer may expire first or the response may arrive first.
     * performAction uses a when_any to distinguish between both cases.
     * 1) The response arrives first. performAction inspects the response:
	 *     a) If the response string starts with "FBK", the server will send new responses.
	 *        performAction starts reading again to receive the next response.
	 *     b) If the response string starts with "RES", the server will not send any new responses.
	 *        performAction cancels the running timer and it leaves the loop.
     * 2) The timer expires first. performAction sends a STOP request to the server
     * and it starts timer of 500 ms. It leaves the loop.
     *
     * @param the timeout to use
     * @return async_task<int> with value 0
     */
	async_task<int> performAction(int timeout)
	{
		// Prepare the GOAL request
		std::string str1 = "GOAL:params-go-here\n";

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

		print(PRI1, "performAction: when_any<async_operation_base> war( { &sr, &st } ) ;\n");
		when_any<async_operation_base> war({ &sr, &st });

		int counter = 0;
		bool done = false;
		while (!done)
		{
			print(PRI1, "performAction: %d ---------------------------\n", counter);

			print(PRI1, "performAction: int i = co_await war;\n");
			int i = co_await war;

			switch (i)
			{
			case 0:
			{
				print(PRI1, "performAction: i = %d: reply has arrived\n", i);
				print(PRI1, "performAction: i = %d: sr.get_result() = %s\n", i, sr.get_result().c_str());
				
				// Distinguish between feedback and result
				std::string header = getHeader(sr.get_result());
				if (header.compare("FBK") == 0)
				{
					counter++;
					print(PRI1, "performAction: i = %d: feedback %d has arrived\n", i, counter);
					print(PRI1, "performAction: i = %d: async_operation<std::string> sr = start_reading();\n", i);
					sr = start_reading();
				}
				else if (header.compare("RES") == 0)
				{
					print(PRI1, "performAction: client_timer.cancel();\n");
					client_timer.cancel();
					print(PRI1, "performAction: i = %d: reply has arrived in time\n", i);
					done = true;
				}
				else
				{
					print(PRI1, "performAction: i = %d, unexpected answer %s received\n", i, header.c_str());
				}
			}
			break;
			case 1: 
			{
				// Timer has expired, send a cancel request to the server
				print(PRI1, "performAction: i = %d: timer has expired, send a cancel request to the server\n", i);

				// Prepare the STOP request
				std::string str1 = "STOP\n";

				// Writing
				print(PRI1, "performAction: i = %d: async_operation<void> sw2 = start_writing(...);\n", i);
				async_operation<void> sw2 = start_writing(str1.c_str(), str1.length() + 1);
				print(PRI1, "performAction: i = %d: co_await sw2;\n", i);
				co_await sw2;

				// Start a timer of 500 ms
				// Timing
				print(PRI1, "performAction: i = %d: st = start_timer(500);\n", i);
				st = start_timer(client_timer, 500);
				
				done = true;
			}
			break;
			default:
				print(PRI1, "performAction: i = %d: should not occur\n", i);
			}
		}

		print(PRI1, "performAction: co_return 0;\n");
		co_return 0;
	}
	
    /**
     * @brief mainflow repeats the following actions 10 times:
     * 1) it connects to the server
     * 2) it calls performAction (see above) to perform the interaction with the server
     * and co_waits its completion.
     * 3) it starts a timer of 100 ms
     * 4) it closes the connection
     *
     * @return async_task<int> with return value 0
     */
	async_task<int> mainflow()
	{
		print(PRI1, "mainflow: begin\n");

		for (int i = 0; i < 10; i++)
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

			// The server waits 1000 ms before sending either a feedback or the response.
			print(PRI1, "mainflow: async_task<int> pA = performAction(%d);\n", (i % 2) ? 20000 : secondtimeout);
			async_task<int> pA = performAction((i % 2) ? 20000 : secondtimeout);
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
	set_priority(0x01);

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
