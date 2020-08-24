/** 
 * @file client3WAny.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement CommClient.
 * This example uses 3 CommClient1 objects.
 * It also uses a wait_any_awaitable type that allows awaiting the completion of 1 of N asychronous operations.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
 
#include "corolib/print.h"
#include "corolib/async_task.h"
#include "corolib/async_operation.h"
#include "corolib/wait_any_awaitable.h"
#include "corolib/commclient.h"

#include "endpoints.h"

using namespace corolib;

const int corolib::priority = 0x0F;

async_task<int> mainflowWA0(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflowWA0: begin\n");

	int counter = 0;
	for (int i = 0; i < 30; i++)
	{
		// Connecting
		print(PRI1, "mainflowWA0: async_operation sc1 = c1.start_connecting();\n");
		async_operation sc1 = c1.start_connecting();
		print(PRI1, "mainflowWA0: async_operation sc2 = c2.start_connecting();\n");
		async_operation sc2 = c2.start_connecting();
		print(PRI1, "mainflowWA0: async_operation sc3 = c3.start_connecting();\n");
		async_operation sc3 = c3.start_connecting();

		print(PRI1, "mainflowWA0: wait_any_awaitable<int> wac( { &sc1, &sc2, &sc3 } );\n");
		wait_any_awaitable<async_operation> wac( { &sc1, &sc2, &sc3 } );
		for (int i = 0; i < 3; i++) {
			print(PRI1, "mainflowWA0: co_await wac;\n");
			co_await wac;
			print(PRI1, "mainflowWA0: wac %d completed\n", i);
		}

		// Writing
		std::string str1 = "This is string ";
		str1 += std::to_string(counter++);
		str1 += " to echo\n";
		std::string str2 = "This is string ";
		str2 += std::to_string(counter++);
		str2 += " to echo\n";
		std::string str3 = "This is string ";
		str3 += std::to_string(counter++);
		str3 += " to echo\n";

		print(PRI1, "mainflowWA0: async_operation sw1 = c1.start_writing(...);\n");
		async_operation sw1 = c1.start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "mainflowWA0: async_operation sw2 = c2.start_writing(...);\n");
		async_operation sw2 = c2.start_writing(str2.c_str(), str2.length() + 1);
		print(PRI1, "mainflowWA0: async_operation sw3 = c3.start_writing(...);\n");
		async_operation sw3 = c3.start_writing(str3.c_str(), str3.length() + 1);

		print(PRI1, "mainflowWA0: wait_all_awaitable<async_operation> waw( { &sw1, &sw2, &sw3 } );\n");
		wait_any_awaitable<async_operation> waw( { &sw1, &sw2, &sw3 } );
		for (int i = 0; i < 3; i++) {
			print(PRI1, "mainflowWA0: int r = co_await waw;\n");
			int r = co_await waw;
			print(PRI1, "mainflowWA0: waw %d completed\n", r);
		}
		
		// Reading
		print(PRI1, "mainflowWA0: async_operation_t<std::string> sr1 = c1.start_reading();\n");
		async_operation_t<std::string> sr1 = c1.start_reading();
		print(PRI1, "mainflowWA0: async_operation_t<std::string> sr2 = c2.start_reading();\n");
		async_operation_t<std::string> sr2 = c2.start_reading();
		print(PRI1, "mainflowWA0: async_operation_t<std::string> sr3 = c3.start_reading();\n");
		async_operation_t<std::string> sr3 = c3.start_reading();

		print(PRI1, "mainflowWA0: wait_any_awaitable<async_operation> war( { &sr1, &sr2, &sr3 } ) ;\n");
		wait_any_awaitable<async_operation> war( { &sr1, &sr2, &sr3 } );
		for (int i = 0; i < 3; i++) {
			print(PRI1, "mainflowWA0: int r = co_await war;\n");
			int r = co_await war;
			print(PRI1, "mainflowWA0: war %d completed\n", r);
		}
		
		print(PRI1, "sr1.get_result() = %s", sr1.get_result().c_str());
		print(PRI1, "sr2.get_result() = %s", sr2.get_result().c_str());
		print(PRI1, "sr3.get_result() = %s", sr3.get_result().c_str());

		// Closing
		print(PRI1, "mainflowWA0: c1.stop();\n");
		c1.stop();
		print(PRI1, "mainflowWA0: c2.stop();\n");
		c2.stop();
		print(PRI1, "mainflowWA0: c3.stop();\n");
		c3.stop();

		print(PRI1, "mainflowWA0: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflowWA0: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflowWA1(std::initializer_list<CommClient*> clients)
{
	print(PRI1, "mainflowWA1: begin\n");

	const int nrClients = 3; //clients.size();

	async_operation asyncsc[nrClients];
	async_operation asyncsw[nrClients];
	async_operation_t<std::string> asyncsr[nrClients];
	std::string results[nrClients];
	std::string str[nrClients];

	int counter = 0;
	for (int i = 0; i < 30; i++)
	{
		int j;

		// Connecting
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		
		print(PRI1, "mainflowWA1: wait_any_awaitable<async_operation> wac(asyncsc, nrClients)\n");
		wait_any_awaitable<async_operation> wac(asyncsc, nrClients);
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "mainflowWA1: int r = co_await wac;\n");
			int r = co_await wac;
			print(PRI1, "mainflowWA1: wac %d completed\n", r);
		}

		// Writing
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: asyncsw[%d] = cl->start_writing(...);\n", j);
			str[j] = "This is string ";
			str[j] += std::to_string(counter++);
			str[j] += " to echo\n";
			asyncsw[j] = cl->start_writing(str[j].c_str(), str[j].length() + 1);
			j++;
		}
		
		print(PRI1, "mainflowWA1: wait_any_awaitable<async_operation> waw(asyncsw, 3)\n");
		wait_any_awaitable<async_operation> waw(asyncsw, 3);
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "mainflowWA1: int r = co_await waw;\n");
			int r = co_await waw;
			print(PRI1, "mainflowWA1: waw %d completed\n", r);
		}
		
		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		
		print(PRI1, "mainflowWA1: wait_any_awaitable<async_operation_t<std::string>> war(asyncsr, 3)\n");
		wait_any_awaitable<async_operation_t<std::string>> war(asyncsr, nrClients);
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "mainflowWA1: int r = co_await war;\n");
			int r = co_await war;
			print(PRI1, "mainflowWA1: war %d completed\n", r);
		}
		
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "asyncsr[%d].get_result() = %s", i, asyncsr[i].get_result().c_str());
		}
		
		// Closing
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: cl->stop();\n");
			cl->stop();
		}

		print(PRI1, "mainflowWA1: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflowWA1: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflowWA2(std::initializer_list<CommClient*> clients)
{
	print(PRI1, "mainflowWA2: begin\n");
	const int nrClients = 3; // clients.size();
	
	int counter = 0;
	for (int i = 0; i < 30; i++)
	{
		async_operation asyncsc[nrClients];
		async_operation asyncsw[nrClients];
		async_operation_t<std::string> asyncsr[3];
		std::string str[nrClients];
		std::string results[nrClients];

		int j;

		// Connecting
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		
		print(PRI1, "mainflowWA2: wait_all_awaitable<async_operation> wac(asyncsc, nrClients)\n");
		wait_any_awaitable<async_operation> wac(asyncsc, nrClients);
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "mainflowWA2: int r = co_await wac;\n");
			int r = co_await wac;
			print(PRI1, "mainflowWA2: wac %d completed\n", r);
		}
		
		// Writing
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: asyncsw[%d] = cl->start_writing(...);\n", j);
			str[j] = "This is string ";
			str[j] += std::to_string(counter++);
			str[j] += " to echo\n";
			asyncsw[j] = cl->start_writing(str[j].c_str(), str[j].length() + 1);
			j++;
		}
		
		print(PRI1, "mainflowWA2: wait_any_awaitable<async_operation> waw(asyncsw, 3)\n");
		wait_any_awaitable<async_operation> waw(asyncsw, 3);
		for (int i = 0; i < 3; i++) {
			print(PRI1, "mainflowWA2: int r = co_await waw;\n");
			int r = co_await waw;
			print(PRI1, "mainflowWA2: waw %d completed\n", r);
		}
		
		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		
		print(PRI1, "mainflowWA2: wait_any_awaitable<async_operation_t<std::string>> war(asyncsr, nrClients)\n");
		wait_any_awaitable<async_operation_t<std::string>> war(asyncsr, nrClients);
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "mainflowWA2: int r = co_await war;\n");
			int r = co_await war;
			print(PRI1, "mainflowWA2: war %d completed\n", r);
		}
		
		for (int i = 0; i < nrClients; i++) {
			print(PRI1, "asyncsr[%d].get_result() = %s", i, asyncsr[i].get_result().c_str());
		}
		
		// Closing
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: cl->stop();\n");
			cl->stop();
		}

		print(PRI1, "mainflowWA2: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflowWA2: co_return 0;\n");
	co_return 0;
}

int main(int argc, char* argv[])
{
    boost::asio::io_context ioContext;

	print(PRI1, "main: CommClient c1(ioContext, ep1);\n");
	CommClient c1(ioContext, ep1);
	print(PRI1, "main: CommClient c2(ioContext, ep1);\n");
	CommClient c2(ioContext, ep1);
	print(PRI1, "main: CommClient c3(ioContext, ep1);\n");
	CommClient c3(ioContext, ep1);

	int selected = 0;
	if (argc == 2)
		selected = atoi(argv[1]);
	if (selected < 0 || selected > 2)
		selected = 0;

	switch (selected) {
	case 0:
	{
		print(PRI1, "main: async_task<int> si = mainflowWA0(c1, c2, c3);\n");
		async_task<int> si0 = mainflowWA0(c1, c2, c3);
	}
	break;
	case 1:
	{
		print(PRI1, "main: async_task<int> si3 = mainflowWA1( {&c1, &c2, &c3} )\n");
		async_task<int> si1 = mainflowWA1({ &c1, &c2, &c3 });
	}
	break;
	case 2:
	{
		print(PRI1, "main: async_task<int> si3 = mainflowWA2( {&c1, &c2, &c3} )\n");
		async_task<int> si2 = mainflowWA2({ &c1, &c2, &c3 });
	}
	break;
	}

	print(PRI1, "main: ioContext.run();\n");
	ioContext.run();

    print(PRI1, "main: return 0;\n");
	return 0;
}
