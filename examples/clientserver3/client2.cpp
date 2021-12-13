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
#include "endpoints.h"

using namespace corolib;

#include "reqresptypes.h"

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

	async_task<Resp1> operation1(Req1)
	{
		std::string str1 = "Req1:params-go-here\n";
		// TODO: Marshall fields of Req1 into string
		
		// Writing
		print(PRI1, "operation1: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation1: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation1: async_operation<std::string> sr = start_reading();\n");
		async_operation<std::string> sr = start_reading();
		print(PRI1, "operation1: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation1: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp1
		
		co_return Resp1{};
	}
	
	async_task<Resp2> operation2(Req2)
	{
		std::string str1 = "Req2:params-go-here\n";
		// TODO: Marshall fields of Req2 into string

		// Writing
		print(PRI1, "operation2: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation2: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation2: async_operation<std::string> sr = start_reading();\n");
		async_operation<std::string> sr = start_reading();
		print(PRI1, "operation2: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation2: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp2
		
		co_return Resp2{};
	}
	
	async_task<Resp3> operation3(Req3)
	{
		std::string str1 = "Req3:params-go-here\n";
		// TODO: Marshall fields of Req3 into string
		
		// Writing
		print(PRI1, "operation3: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation3: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation3: async_operation<std::string> sr = start_reading();\n");
		async_operation<std::string> sr = start_reading();
		print(PRI1, "operation3: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation3: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp3
		
		co_return Resp3{};
	}
	
	async_task<Resp4> operation4(Req4)
	{
		std::string str1 = "Req4:params-go-here\n";
		// TODO: Marshall fields of Req4 into string
		
		// Writing
		print(PRI1, "operation4: async_operation<void> sw = start_writing(...);\n");
		async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation4: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation4: async_operation<std::string> sr = start_reading();\n");
		async_operation<std::string> sr = start_reading();
		print(PRI1, "operation4: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation4: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp4
		
		co_return Resp4{};
	}
	
	struct process_info_t
	{
		int iteration = -1;
		int step = -1;
	};

	static const int NR_ITERATIONS = 10;

	async_task<void> mainflow(process_info_t &process_info)
	{
		print(PRI1, "mainflow: begin\n");

		for (int i = 0; i < NR_ITERATIONS; i++)
		{
			print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", i);
			process_info.iteration = i;
			process_info.step = 0;

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

			process_info.step = 1;
			print(PRI1, "mainflow: async_task<Resp1> res1 = operation1(Req1{})\n");
			async_task<Resp1> res1 = operation1(Req1{});
			print(PRI1, "mainflow: Resp1 resp1 = co_await res1\n");
			Resp1 resp1 = co_await res1;

			process_info.step = 2;
			print(PRI1, "mainflow: async_task<Resp2> res2 = operation2(Req2{})\n");
			async_task<Resp2> res2 = operation2(Req2{});
			print(PRI1, "mainflow: Resp2 resp2 = co_await res2\n");
			Resp2 resp2 = co_await res2;

			process_info.step = 3;
			print(PRI1, "mainflow: async_task<Resp3> res3 = operation3(Req3{})\n");
			async_task<Resp3> res3 = operation3(Req3{});
			print(PRI1, "mainflow: Resp3 resp3 = co_await res3\n");
			Resp3 resp3 = co_await res3;

			process_info.step = 4;
			print(PRI1, "mainflow: async_task<Resp4> res4 = operation4(Req4{})\n");
			async_task<Resp4> res4 = operation4(Req4{});
			print(PRI1, "mainflow: Resp4 resp4 = co_await res4\n");
			Resp4 resp4 = co_await res4;
			
			// Delaying
			process_info.step = 5;
			steady_timer client_timer(ioContext);
			print(PRI1, "mainflow: async_operation<void> st = start_timer(1000);\n");
			async_operation<void> st = start_timer(client_timer, 1000);
			print(PRI1, "mainflow: co_await st;\n");
			co_await st;

			// Closing
			print(PRI1, "mainflow: stop();\n");
			stop();
		}

		print(PRI1, "mainflow: co_return;\n");
		co_return;
	}

	/**
	 * @brief Monitors the progress of mainflow(process_info_t);
	 */
	async_task<void> mainflow()
	{
		process_info_t process_info;
		async_task<void> mainflow_task = mainflow(process_info);

		steady_timer client_timer(ioContext);
		while (process_info.iteration < NR_ITERATIONS)
		{
			print(PRI1, "mainflow: process_info = {%d, %d};\n", process_info.iteration, process_info.step);
			async_operation<void> st = start_timer(client_timer, 50);
			co_await st;
		}

		co_await mainflow_task;
		co_return;
	}
};

int main()
{
	set_priority(0x01);

	print(PRI1, "main: ClientApp c1(ioContext, ep);\n");
	ClientApp c1(ioContext, ep);

	print(PRI1, "main: async_task<int> si = mainflow(c1);\n");
	async_task<void> si = c1.mainflow();

	print(PRI1, "main: ioContext.run();\n");
	ioContext.run();

	print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
	std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
	return 0;
}
