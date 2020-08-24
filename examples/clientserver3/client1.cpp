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

	async_task<Resp1> operation1(Req1)
	{
		std::string str1 = "Req1\n";
		// TODO: Marshall fields of Req1 into string
		
		// Writing
		print(PRI1, "operation1: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation1: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation1: async_operation sr = start_reading();\n");
		async_operation_t<std::string> sr = start_reading();
		print(PRI1, "operation1: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation1: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp1
		
		co_return Resp1{};
	}
	
	async_task<Resp2> operation2(Req2)
	{
		std::string str1 = "Req2\n";
		// TODO: Marshall fields of Req2 into string

		// Writing
		print(PRI1, "operation2: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation2: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation2: async_operation sr = start_reading();\n");
		async_operation_t<std::string> sr = start_reading();
		print(PRI1, "operation2: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation2: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp2
		
		co_return Resp2{};
	}
	
	async_task<Resp3> operation3(Req3)
	{
		std::string str1 = "Req3\n";
		// TODO: Marshall fields of Req3 into string
		
		// Writing
		print(PRI1, "operation3: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation3: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation3: async_operation sr = start_reading();\n");
		async_operation_t<std::string> sr = start_reading();
		print(PRI1, "operation3: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation3: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp3
		
		co_return Resp3{};
	}
	
	async_task<Resp4> operation4(Req4)
	{
		std::string str1 = "Req4\n";
		// TODO: Marshall fields of Req4 into string
		
		// Writing
		print(PRI1, "operation4: async_operation sw = start_writing(...);\n");
		async_operation sw = start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "operation4: co_await sw;\n");
		co_await sw;

		// Reading
		print(PRI1, "operation4: async_operation sr = start_reading();\n");
		async_operation_t<std::string> sr = start_reading();
		print(PRI1, "operation4: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "operation4: strout = %s", strout.c_str());
		// TODO: Unmarshall string into fields of Resp4
		
		co_return Resp4{};
	}
			
	async_task<int> mainflow()
	{
		print(PRI1, "mainflow: begin\n");
		int counter = 0;

		for (int i = 0; i < 20; i++)
		{
			// Connecting
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

			print(PRI1, "mainflow: async_task<Resp1> res1 = operation1(Req1{})\n");
			async_task<Resp1> res1 = operation1(Req1{});
			print(PRI1, "mainflow: Resp1 resp1 = co_await res1\n");
			Resp1 resp1 = co_await res1;

			print(PRI1, "mainflow: async_task<Resp2> res2 = operation2(Req2{})\n");
			async_task<Resp2> res2 = operation2(Req2{});
			print(PRI1, "mainflow: Resp2 resp2 = co_await res2\n");
			Resp2 resp2 = co_await res2;

			print(PRI1, "mainflow: async_task<Resp3> res3 = operation3(Req3{})\n");
			async_task<Resp3> res3 = operation3(Req3{});
			print(PRI1, "mainflow: Resp3 resp3 = co_await res3\n");
			Resp3 resp3 = co_await res3;

			print(PRI1, "mainflow: async_task<Resp4> res4 = operation4(Req4{})\n");
			async_task<Resp4> res4 = operation4(Req4{});
			print(PRI1, "mainflow: Resp4 resp4 = co_await res4\n");
			Resp4 resp4 = co_await res4;
			
			// Delaying
			steady_timer client_timer(ioContext);
			print(PRI1, "mainflow: async_operation st = start_timer(1000);\n");
			async_operation st = start_timer(client_timer, 1000);
			print(PRI1, "mainflow: co_await st;\n");
			co_await st;

			// Closing
			print(PRI1, "mainflow: stop();\n");
			stop();
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

	print(PRI1, "main: ioContext.run();\n");
	ioContext.run();

	print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
	std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
	return 0;
}
