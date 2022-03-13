/** 
 * @file client3WA.cpp
 * @brief
 * Example of a client application.
 * This client application uses 3 CommClient objects.
 * It uses a wait_all object that allows awaiting the completion of N asychronous operations.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
 
#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/wait_all_awaitable.h>
#include <corolib/commclient.h>

#include "endpoints.h"

using namespace corolib;

async_task<int> mainflowWA0(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflowWA0: begin\n");

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflowWA0: %d ------------------------------------------------------------------\n", i);

		// Connecting
		print(PRI1, "mainflowWA0: async_operation<void> sc1 = c1.start_connecting();\n");
		async_operation<void> sc1 = c1.start_connecting();
		print(PRI1, "mainflowWA0: async_operation<void> sc2 = c2.start_connecting();\n");
		async_operation<void> sc2 = c2.start_connecting();
		print(PRI1, "mainflowWA0: async_operation<void> sc3 = c3.start_connecting();\n");
		async_operation<void> sc3 = c3.start_connecting();

		print(PRI1, "mainflowWA0: wait_all<async_operation<void>> wac( { &sc1, &sc2, &sc3 } );\n");
		wait_all<async_operation<void>> wac( { &sc1, &sc2, &sc3 } );
		print(PRI1, "mainflowWA0: co_await wac;\n");
		co_await wac;

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

		print(PRI1, "mainflowWA0: async_operation<void> sw1 = c1.start_writing(...);\n");
		async_operation<void> sw1 = c1.start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "mainflowWA0: async_operation<void> sw2 = c2.start_writing(...);\n");
		async_operation<void> sw2 = c2.start_writing(str2.c_str(), str2.length() + 1);
		print(PRI1, "mainflowWA0: async_operation<void> sw3 = c3.start_writing(...);\n");
		async_operation<void> sw3 = c3.start_writing(str3.c_str(), str3.length() + 1);

		print(PRI1, "mainflowWA0: wait_all<async_operation<void>> waw( { &sw1, &sw2, &sw3 } );\n");
        wait_all<async_operation<void>> waw( { &sw1, &sw2, &sw3 } );
		print(PRI1, "mainflowWA0: co_await waw;\n");
		co_await waw;

		// Reading
		print(PRI1, "mainflowWA0: async_operation<std::string> sr1 = c1.start_reading();\n");
		async_operation<std::string> sr1 = c1.start_reading();
		print(PRI1, "mainflowWA0: async_operation<std::string> sr2 = c2.start_reading();\n");
		async_operation<std::string> sr2 = c2.start_reading();
		print(PRI1, "mainflowWA0: async_operation<std::string> sr3 = c3.start_reading();\n");
		async_operation<std::string> sr3 = c3.start_reading();

		print(PRI1, "mainflowWA0: wait_all<async_operation<std::string>> war( { &sr1, &sr2, &sr3 } ) ;\n");
        wait_all<async_operation<std::string>> war( { &sr1, &sr2, &sr3 } );
		print(PRI1, "mainflowWA0: co_await war;\n");
		co_await war;
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

	async_operation<void> asyncsc[nrClients];
	async_operation<void> asyncsw[nrClients];
	async_operation<std::string> asyncsr[nrClients];
	std::string results[nrClients];
	std::string str[nrClients];

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflowWA1: %d ------------------------------------------------------------------\n", i);

		int j;

		// Connecting
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		print(PRI1, "mainflowWA1: wait_all<async_operation<void>> wac(asyncsc, 3)\n");
        wait_all<async_operation<void>> wac(asyncsc, 3);
		print(PRI1, "mainflowWA1: co_await wac;\n");
		co_await wac;

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
		print(PRI1, "mainflowWA1: wait_all<async_operation<void>> waw(asyncsw, 3)\n");
        wait_all<async_operation<void>> waw(asyncsw, 3);
		print(PRI1, "mainflowWA1: co_await waw;\n");
		co_await waw;

		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA1: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		print(PRI1, "mainflowWA1: wait_all<async_operation<std::string>> war(asyncsr, 3)\n");
		wait_all<async_operation<std::string>> war(asyncsr, 3);
		print(PRI1, "mainflowWA1: co_await war;\n");
		co_await war;
		for (int i = 0; i < nrClients; i++)
			print(PRI1, "asyncsr[%d].get_result() = %s", i, asyncsr[i].get_result().c_str());

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
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflowWA2: %d ------------------------------------------------------------------\n", i);

		async_operation<void> asyncsc[nrClients];
		async_operation<void> asyncsw[nrClients];
		async_operation<std::string> asyncsr[3];
		std::string str[nrClients];
		std::string results[nrClients];

		int j;

		// Connecting
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		print(PRI1, "mainflowWA2: wait_all<async_operation<void>> wac(asyncsc, 3)\n");
        wait_all<async_operation<void>> wac(asyncsc, 3);
		print(PRI1, "mainflowWA2: co_await wac;\n");
		co_await wac;

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
		print(PRI1, "mainflowWA2: wait_all<async_operation<void>> waw(asyncsw, 3)\n");
        wait_all<async_operation<void>> waw(asyncsw, 3);
		print(PRI1, "mainflowWA2: co_await waw;\n");
		co_await waw;

		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflowWA2: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		print(PRI1, "mainflowWA2: wait_all<async_operation<std::string>> war(asyncsr, 3)\n");
        wait_all<async_operation<std::string>> war(asyncsr, 3);
		print(PRI1, "mainflowWA2: co_await war;\n");
		co_await war;
		for (int i = 0; i < nrClients; i++)
			print(PRI1, "asyncsr[%d].get_result() = %s", i, asyncsr[i].get_result().c_str());

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

async_task<int> mainflowOneClient(CommClient& c1, int instance, int counter)
{
	print(PRI1, "mainflowOneClient: %d: begin\n", instance);

	// Connecting
	print(PRI1, "mainflowOneClient: %d: async_operation<void> sc1 = c1.start_connecting();\n", instance);
	async_operation<void> sc1 = c1.start_connecting();
	print(PRI1, "mainflowOneClient: %d: co_await sc1;\n", instance);
	co_await sc1;

	// Writing
	std::string str = "This is string ";
	str += std::to_string(counter);
	str += " to echo\n";
	print(PRI1, "mainflowOneClient: %d: async_operation<void> sw1 = c1.start_writing(...);\n", instance);
	async_operation<void> sw1 = c1.start_writing(str.c_str(), str.length() + 1);
	print(PRI1, "mainflowOneClient: %d: co_await sw1;\n", instance);
	co_await sw1;

	// Reading
	print(PRI1, "mainflowOneClient: %d: async_operation<std::string> sr1 = c1.start_reading();\n", instance);
	async_operation<std::string> sr1 = c1.start_reading('\n');
	print(PRI1, "mainflowOneClient: %d: std::string strout = co_await sr1;\n", instance);
	std::string strout = co_await sr1;
	print(PRI1, "mainflowOneClient: %d: strout = %s", instance, strout.c_str());

	// Closing
	print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance );
	c1.stop();

	co_return 0;
}

async_task<int> mainflowWA3(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflowWA3: begin\n");

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflowWA3: %d ------------------------------------------------------------------\n", i);

		print(PRI1, "mainflowWA3: mainflowOneClient(c1, 0, counter++);\n");
		async_task<int> tc1 = mainflowOneClient(c1, 0, counter++);
		print(PRI1, "mainflowWA3: mainflowOneClient(c2, 1, counter++);\n");
		async_task<int> tc2 = mainflowOneClient(c2, 1, counter++);
		print(PRI1, "mainflowWA3: mainflowOneClient(c3, 2, counter++);\n");
		async_task<int> tc3 = mainflowOneClient(c3, 2, counter++);

		print(PRI1, "mainflowWA3: wait_all<async_task<int>> wat({ &tc1, &tc2, &tc3 });\n");
        wait_all<async_task<int>> wat({ &tc1, &tc2, &tc3 });
		print(PRI1, "mainflowWA3: co_await wat;\n");
		co_await wat;

		print(PRI1, "mainflowWA3: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	print(PRI1, "mainflowWA3: co_return 0;\n");
	co_return 0;
}

void mainflowX(CommClient& c1, CommClient& c2, CommClient& c3, int selected)
{
	switch (selected) {
	case 0:
	{
		print(PRI1, "mainflowX: async_task<int> si0 = mainflowWA0(c1, c2, c3);\n");
		async_task<int> si0 = mainflowWA0(c1, c2, c3);
	}
	break;
	case 1:
	{
		print(PRI1, "mainflowX: async_task<int> si1 = mainflowWA1( {&c1, &c2, &c3} )\n");
		async_task<int> si1 = mainflowWA1({ &c1, &c2, &c3 });
	}
	break;
	case 2:
	{
		print(PRI1, "mainflowX: async_task<int> si2 = mainflowWA2( {&c1, &c2, &c3} )\n");
		async_task<int> si2 = mainflowWA2({ &c1, &c2, &c3 });
	}
	break;
	case 3:
	{
		print(PRI1, "mainflowX: async_task<int> si3 = mainflowWA3(c1, c2, c3)\n");
		async_task<int> si3 = mainflowWA3(c1, c2, c3);
	}
	break;
	}
}

async_task<int> mainflowAll(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflowAll: async_task<int> si0 = mainflowWA0(c1, c2, c3);\n");
	async_task<int> si0 = mainflowWA0(c1, c2, c3);
	print(PRI1, "mainflowAll: co_await si0;\n");
	co_await si0;

	print(PRI1, "mainflowAll: async_task<int> si1 = mainflowWA1( {&c1, &c2, &c3} )\n");
	async_task<int> si1 = mainflowWA1({ &c1, &c2, &c3 });
	print(PRI1, "mainflowAll: co_await si1;\n");
	co_await si1;

	print(PRI1, "mainflowAll: async_task<int> si2 = mainflowWA2( {&c1, &c2, &c3} )\n");
	async_task<int> si2 = mainflowWA2({ &c1, &c2, &c3 });
	print(PRI1, "mainflowAll: co_await si2;\n");
	co_await si2;

	print(PRI1, "mainflowAll: async_task<int> si3 = mainflowWA3(c1, c2, c3} )\n");
	async_task<int> si3 = mainflowWA3(c1, c2, c3);
	print(PRI1, "mainflowAll: co_await si3;\n");
	co_await si3;

	print(PRI1, "mainflowAll: co_return 0;\n");
	co_return 0;
}

int main(int argc, char* argv[])
{
	set_priority(0x01);

    boost::asio::io_context ioContext;

	print(PRI1, "main: CommClient c1(ioContext, ep1);\n");
	CommClient c1(ioContext, ep1);
	print(PRI1, "main: CommClient c2(ioContext, ep1);\n");
	CommClient c2(ioContext, ep1);
	print(PRI1, "main: CommClient c3(ioContext, ep1);\n");
	CommClient c3(ioContext, ep1);

	if (argc == 2)
	{
		int selected = 0;
		selected = atoi(argv[1]);
		if (selected < 0 || selected > 3)
		{
			print(PRI1, "main: selection must be in the range [0..3]\n");
			return 0;
		}
		print(PRI1, "main: mainflowX(c1, c2, c3, selected);\n");
		mainflowX(c1, c2, c3, selected);
	}
	else
	{
		print(PRI1, "main: async_task<int> si = mainflowAll(c1, c2, c3);\n");
		async_task<int> si = mainflowAll(c1, c2, c3);
	}
	
	print(PRI1, "main: before ioContext.run();\n");
	ioContext.run();
	print(PRI1, "main: after ioContext.run();\n");

	print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
	std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
	return 0;
}
