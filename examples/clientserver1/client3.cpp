/** 
 * @file client3.cpp
 * @brief
 * Example of a client application.
 * This client application uses 3 CommClient objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio.hpp>

#include "corolib/print.h"
#include "corolib/async_task.h"
#include "corolib/oneway_task.h"
#include "corolib/auto_reset_event.h"
#include "corolib/async_operation.h"
#include "corolib/commclient.h"

#include "endpoints.h"

using namespace corolib;

oneway_task mainflowOneClient(CommClient& c1, auto_reset_event& are, int instance, int counter)
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
	print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance);
	c1.stop();

	are.resume();
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
	print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance);
	c1.stop();

	co_return 0;
}

async_task<int> mainflow0(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflow0: begin\n");

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow0: %d ------------------------------------------------------------------\n", i);

		// Connecting
		print(PRI1, "mainflow0: async_operation<void> sc1 = c1.start_connecting();\n");
		async_operation<void> sc1 = c1.start_connecting();
		print(PRI1, "mainflow0: async_operation<void> sc2 = c2.start_connecting();\n");
		async_operation<void> sc2 = c2.start_connecting();
		print(PRI1, "mainflow0: async_operation<void> sc3 = c3.start_connecting();\n");
		async_operation<void> sc3 = c3.start_connecting();

		print(PRI1, "mainflow0: co_await sc1;\n");
		co_await sc1;
		print(PRI1, "mainflow0: co_await sc2;\n");
		co_await sc2;
		print(PRI1, "mainflow0: co_await sc3;\n");
		co_await sc3;

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

		print(PRI1, "mainflow0: async_operation<void> sw1 = c1.start_writing(...);\n");
		async_operation<void> sw1 = c1.start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "mainflow0: async_operation<void> sw2 = c2.start_writing(...);\n");
		async_operation<void> sw2 = c2.start_writing(str2.c_str(), str2.length() + 1);
		print(PRI1, "mainflow0: async_operation<void> sw3 = c3.start_writing(...);\n");
		async_operation<void> sw3 = c3.start_writing(str3.c_str(), str3.length() + 1);

#if 1
		print(PRI1, "mainflow0: co_await sw1;\n");
		co_await sw1;
		print(PRI1, "mainflow0: co_await sw2;\n");
		co_await sw2;
		print(PRI1, "mainflow0: co_await sw3;\n");
		co_await sw3;
#else
		print(PRI1, "mainflow0: co_await sw2;\n");
		co_await sw2;
		print(PRI1, "mainflow0: co_await sw1;\n");
		co_await sw1;
		print(PRI1, "mainflow0: co_await sw3;\n");
		co_await sw3;
#endif
		// Reading
		print(PRI1, "mainflow0: async_operation<std::string> sr1 = c1.start_reading();\n");
		async_operation<std::string> sr1 = c1.start_reading('\n');
		print(PRI1, "mainflow0: async_operation<std::string> sr2 = c2.start_reading();\n");
		async_operation<std::string> sr2 = c2.start_reading('\n');
		print(PRI1, "mainflow0: async_operation<std::string> sr3 = c3.start_reading();\n");
		async_operation<std::string> sr3 = c3.start_reading('\n');

		print(PRI1, "mainflow0: std::string strout1 = co_await sr1;\n");
		std::string strout1 = co_await sr1;
		print(PRI1, "mainflow0: strout = %s", strout1.c_str());
		print(PRI1, "mainflow0: std::string strout2 = co_await sr2;\n");
		std::string strout2 = co_await sr2;
		print(PRI1, "mainflow0: strout = %s", strout2.c_str());
		print(PRI1, "mainflow0: std::string strout3 = co_await sr3;\n");
		std::string strout3 = co_await sr3;
		print(PRI1, "mainflow0: strout = %s", strout3.c_str());

		print(PRI1, "mainflow0: c1.stop();\n");
		c1.stop();
		print(PRI1, "mainflow0: c2.stop();\n");
		c2.stop();
		print(PRI1, "mainflow0: c3.stop();\n");
		c3.stop();

		print(PRI1, "mainflow0: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflow0: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow1(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflow1: begin\n");

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow1: %d ------------------------------------------------------------------\n", i);

		auto_reset_event are1, are2, are3;
	
		print(PRI1, "mainflow1: mainflowOneClient(c1, are1, 0, counter++);\n");
		(void) mainflowOneClient(c1, are1, 0, counter++);
		print(PRI1, "mainflow1: mainflowOneClient(c2, are2, 1, counter++);\n");
		(void) mainflowOneClient(c2, are2, 1, counter++);
		print(PRI1, "mainflow1: mainflowOneClient(c3, are3, 2, counter++);\n");
		(void)mainflowOneClient(c3, are3, 2,  counter++);

		print(PRI1, "mainflow1: co_await are1;\n");
		co_await are1;
		print(PRI1, "mainflow1: co_await are2;\n");
		co_await are2;
		print(PRI1, "mainflow1: co_await are3;\n");
		co_await are3;
		
		print(PRI1, "mainflow1: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	print(PRI1, "mainflow1: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow2(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflow2: begin\n");

	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow2: %d ------------------------------------------------------------------\n", i);
		
		print(PRI1, "mainflow2: mainflowOneClient(c1, 0, counter++);\n");
		async_task<int> tc1 = mainflowOneClient(c1, 0, counter++);
		print(PRI1, "mainflow2: mainflowOneClient(c2, 1, counter++);\n");
		async_task<int> tc2 = mainflowOneClient(c2, 1, counter++);
		print(PRI1, "mainflow1: mainflowOneClient(c3, 2, counter++);\n");
		async_task<int> tc3 = mainflowOneClient(c3, 2, counter++);

		print(PRI1, "mainflow2: co_await tc1;\n");
		co_await tc1;
		print(PRI1, "mainflow2: co_await tc1;\n");
		co_await tc2;
		print(PRI1, "mainflow2: co_await tc1;\n");
		co_await tc3;

		print(PRI1, "mainflow2: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	print(PRI1, "mainflow2: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow3(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflow3: begin\n");

	const int nrClients = 3;
	int counter = 0;
	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow3: %d ------------------------------------------------------------------\n", i);

		async_operation<void> asyncsc[nrClients];
		async_operation<void> asyncsw[nrClients];
		async_operation<std::string> asyncsr[nrClients];
		std::string results[nrClients];

		int j;

		// Connecting
		print(PRI1, "mainflow3: asyncsc[0] = c1.start_connecting();\n");
		asyncsc[0] = c1.start_connecting();
		print(PRI1, "mainflow3: asyncsc[1] = c2.start_connecting();\n");
		asyncsc[1] = c2.start_connecting();
		print(PRI1, "mainflow3: asyncsc[2] = c3.start_connecting();\n");
		asyncsc[2] = c3.start_connecting();

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow3: co_await asyncsc[%d];\n", j);
			co_await asyncsc[j];
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

		print(PRI1, "mainflow3: asyncsw[0] = c1.start_writing(...);\n");
		asyncsw[0] = c1.start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "mainflow3: asyncsw[1] = c2.start_writing(...);\n");
		asyncsw[1] = c2.start_writing(str2.c_str(), str2.length() + 1);
		print(PRI1, "mainflow3: asyncsw[2] = c3.start_writing(...);\n");
		asyncsw[2] = c3.start_writing(str3.c_str(), str3.length() + 1);

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow3: co_await asyncsw[%d];\n", j);
			co_await asyncsw[j];
		}

		// Reading
		print(PRI1, "mainflow3: asyncsr[0] = c1.start_reading();\n");
		asyncsr[0] = c1.start_reading();
		print(PRI1, "mainflow3: asyncsr[1] = c2.start_reading();\n");
		asyncsr[1] = c2.start_reading();
		print(PRI1, "mainflow3: asyncsr[2] = c3.start_reading();\n");
		asyncsr[2] = c3.start_reading();

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow3: results[%d] = co_await asyncsr[%d];\n", j, j);
			results[j] = co_await asyncsr[j];
		}

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow3: results[%d] = %s", j, results[j].c_str());
		}

		print(PRI1, "mainflow3: c1.stop();\n");
		c1.stop();
		print(PRI1, "mainflow3: c2.stop();\n");
		c2.stop();
		print(PRI1, "mainflow3: c3.stop();\n");
		c3.stop();

		print(PRI1, "mainflow3: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflow3: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow4(CommClient& c1, CommClient& c2, CommClient& c3)
{
	print(PRI1, "mainflow4: begin\n");

	const int nrClients = 3;
	async_operation<void> asyncsc[nrClients];
	async_operation<void> asyncsw[nrClients];
	async_operation<std::string> asyncsr[nrClients];
	std::string results[nrClients];
	int counter = 0;

	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow4: %d ------------------------------------------------------------------\n", i);

		int j;

		// Connecting
		print(PRI1, "mainflow4: asyncsc[0] = c1.start_connecting();\n");
		asyncsc[0] = c1.start_connecting();
		print(PRI1, "mainflow4: asyncsc[1] = c2.start_connecting();\n");
		asyncsc[1] = c2.start_connecting();
		print(PRI1, "mainflow4: asyncsc[2] = c3.start_connecting();\n");
		asyncsc[2] = c3.start_connecting();

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow4: co_await asyncsc[%d];\n", j);
			co_await asyncsc[j];
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

		print(PRI1, "mainflow4: asyncsw[0] = c1.start_writing(...);\n");
		asyncsw[0] = c1.start_writing(str1.c_str(), str1.length() + 1);
		print(PRI1, "mainflow4: asyncsw[1] = c2.start_writing(...);\n");
		asyncsw[1] = c2.start_writing(str2.c_str(), str2.length() + 1);
		print(PRI1, "mainflow4: asyncsw[2] = c3.start_writing(...);\n");
		asyncsw[2] = c3.start_writing(str3.c_str(), str3.length() + 1);

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow4: co_await asyncsw[%d];\n", j);
			co_await asyncsw[j];
		}

		// Reading
		print(PRI1, "mainflow4: asyncsr[0] = c1.start_reading();\n");
		asyncsr[0] = c1.start_reading();
		print(PRI1, "mainflow4: asyncsr[1] = c2.start_reading();\n");
		asyncsr[1] = c2.start_reading();
		print(PRI1, "mainflow4: asyncsr[2] = c3.start_reading();\n");
		asyncsr[2] = c3.start_reading();

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow4: results[%d] = co_await asyncsr[%d];\n", j, j);
			results[j] = co_await asyncsr[j];
		}

		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow4: results[%d] = %s", j, results[j].c_str());
		}

		print(PRI1, "mainflow4: c1.stop();\n");
		c1.stop();
		print(PRI1, "mainflow4: c2.stop();\n");
		c2.stop();
		print(PRI1, "mainflow4: c3.stop();\n");
		c3.stop();

		print(PRI1, "mainflow4: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflow4: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow5(std::initializer_list<CommClient*> clients)
{
	print(PRI1, "mainflow5: begin\n");

	const int nrClients = 3;
	int counter = 0;

	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow5: %d ------------------------------------------------------------------\n", i);

		async_operation<void> asyncsc[nrClients];
		async_operation<void> asyncsw[nrClients];
		async_operation<std::string> asyncsr[nrClients];
		std::string results[nrClients];

		std::string str[nrClients];
		int j;
		
		// Connecting
		j = 0;
		for (CommClient* cl: clients) {
			print(PRI1, "mainflow5: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow5: co_await asyncsc[%d];\n", j);
			co_await asyncsc[j];
		}

		// Writing
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow5: asyncsw[%d] = cl->start_writing(...);\n", j);
			str[j] = "This is string ";
			str[j] += std::to_string(counter++);
			str[j] += " to echo\n";
			asyncsw[j] = cl->start_writing(str[j].c_str(), str[j].length() + 1);
			j++;
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow5: co_await asyncsw[%d];\n", j);
			co_await asyncsw[j];
		}

		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow5: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow5: results[%d] = co_await asyncsr[%d];\n", j, j);
			results[j] = co_await asyncsr[j];
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow5: results[%d] = %s", j, results[j].c_str());
		}

		// Closing
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow5: cl->stop();\n");
			cl->stop();
		}

		print(PRI1, "mainflow5: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflow5: co_return 0;\n");
	co_return 0;
}

async_task<int> mainflow6(std::initializer_list<CommClient*> clients)
{
	print(PRI1, "mainflow6: begin\n");

	const int nrClients = 3;
	async_operation<void> asyncsc[nrClients];
	async_operation<void> asyncsw[nrClients];
	async_operation<std::string> asyncsr[nrClients];
	std::string results[nrClients];

	int counter = 0;

	for (int i = 0; i < 40; i++)
	{
		print(PRI1, "mainflow6: %d ------------------------------------------------------------------\n", i);

		std::string str[nrClients];
		int j;

		// Connecting
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow6: asyncsc[%d] = cl->start_connecting();\n", j);
			asyncsc[j++] = cl->start_connecting();
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow6: co_await asyncsc[%d];\n", j);
			co_await asyncsc[j];
		}

		// Writing
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow6: asyncsw[%d] = cl->start_writing(...);\n", j);
			str[j] = "This is string ";
			str[j] += std::to_string(counter++);
			str[j] += " to echo\n";
			asyncsw[j] = cl->start_writing(str[j].c_str(), str[j].length() + 1);
			j++;
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow6: co_await asyncsw[%d];\n", j);
			co_await asyncsw[j];
		}

		// Reading
		j = 0;
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow6: asyncsr[%d] = cl->start_reading();\n", j);
			asyncsr[j++] = cl->start_reading();
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow6: results[%d] = co_await asyncsr[%d];\n", j, j);
			results[j] = co_await asyncsr[j];
		}
		for (j = 0; j < nrClients; j++) {
			print(PRI1, "mainflow6: results[%d] = %s", j, results[j].c_str());
		}

		// Closing
		for (CommClient* cl : clients) {
			print(PRI1, "mainflow6: cl->stop();\n");
			cl->stop();
		}

		print(PRI1, "mainflow6: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	print(PRI1, "mainflow6: co_return 0;\n");
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

	int selected = 0;
	if (argc == 2)
		selected = atoi(argv[1]);
	if (selected < 0 || selected > 6) {
		print(PRI1, "main: selection must be in the range [0..6]\n");
		return 0;
	}

	switch (selected) {
	case 0:
	{
		print(PRI1, "main: before async_task<int> si0 = mainflow0(c1, c2, c3);\n");
		async_task<int> si0 = mainflow0(c1, c2, c3);
		print(PRI1, "main: after async_task<int> si0 = mainflow0(c1, c2, c3);\n");
	}
	break;
	case 1:
	{
		print(PRI1, "main: before async_task<int> si1 = mainflow1(c1, c2, c3);\n");
		async_task<int> si1 = mainflow1(c1, c2, c3);
		print(PRI1, "main: after async_task<int> si1 = mainflow1(c1, c2, c3);\n");
	}
	break;
	case 2:
	{
		print(PRI1, "main: before async_task<int> si2 = mainflow2(c1, c2, c3);\n");
		async_task<int> si2 = mainflow2(c1, c2, c3);
		print(PRI1, "main: after async_task<int> si2 = mainflow2(c1, c2, c3);\n");
	}
	break;
	case 3:
	{
		print(PRI1, "main: before async_task<int> si3 = mainflow3(c1, c2, c3);\n");
		async_task<int> si3 = mainflow3(c1, c2, c3);
		print(PRI1, "main: after async_task<int> si3 = mainflow3(c1, c2, c3);\n");
	}
	break;
	case 4:
	{
		print(PRI1, "main: before async_task<int> si4 = mainflow4(c1, c2, c3);\n");
		async_task<int> si4 = mainflow4(c1, c2, c3);
		print(PRI1, "main: after async_task<int> si4 = mainflow4(c1, c2, c3);\n");
	}
	break;
	case 5:
	{
		print(PRI1, "main: before async_task<int> si5 = mainflow5({&c1, &c2, &c3})\n");
		async_task<int> si5 = mainflow5({&c1, &c2, &c3});
		print(PRI1, "main: after async_task<int> si5 = mainflow5({&c1, &c2, &c3})\n");
	}
	break;
	case 6:
	{
		print(PRI1, "main: before async_task<int> si6 = mainflow6({&c1, &c2, &c3})\n");
		async_task<int> si6 = mainflow6({ &c1, &c2, &c3 });
		print(PRI1, "main: after async_task<int> si6 = mainflow6({&c1, &c2, &c3})\n");
	}
	break;
	}

	print(PRI1, "main: before ioContext.run();\n");
	ioContext.run();
	print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: return 0;\n");
	return 0;
}
