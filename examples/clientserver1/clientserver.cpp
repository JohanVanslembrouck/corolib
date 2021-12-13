/** 
 * @file client3WA.cpp
 * @brief
 * Example of a combined client and server application.
 * The application is server for client1, client3 and client3WA.
 * The application is client for the server application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <boost/asio/signal_set.hpp>

#include "corolib/print.h"
#include "corolib/oneway_task.h"
#include "corolib/async_task.h"
#include "corolib/async_operation.h"
#include "corolib/commcore.h"
#include "corolib/commclient.h"
#include "corolib/commserver.h"

#include "endpoints.h"

using namespace corolib;

std::atomic_bool stop{false};

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

boost::asio::io_context ioContextSignal;
boost::asio::io_context ioContextServer;

class ClientServerApp : public CommServer
{
public:
	ClientServerApp(
		boost::asio::io_context& ioContext,
		unsigned short CommService)
		: CommServer(ioContext, CommService)
	{
		print(PRI1, "ClientServerApp::ClientServerApp(...)\n");
	}

	oneway_task mainflow_client(std::string str)
	{
		CommClient commClient(ioContextServer, ep2);

		print(PRI1, "mainflow_client: begin\n");
		for (int i = 0; i < 2; i++)
		{
			// Connecting
			print(PRI1, "mainflow_client: async_operation<void> sc = commClient.start_connecting();\n");
			async_operation<void> sc = commClient.start_connecting();
			print(PRI1, "mainflow_client: co_await sc;\n");
			co_await sc;

			// Writing
			print(PRI1, "mainflow_client: async_operation<void> sw = commClient.start_writing(...);\n");
			async_operation<void> sw = commClient.start_writing(str.c_str(), str.length() + 1);
			print(PRI1, "mainflow_client: co_await sw;\n");
			co_await sw;

			// Reading
			print(PRI1, "mainflow_client: async_operation<std::string> sr = commClient.start_reading();\n");
			async_operation<std::string> sr = commClient.start_reading();
			print(PRI1, "mainflow_client: std::string strout = co_await sr;\n");
			std::string strout = co_await sr;
			print(PRI1, "mainflow_client: strout = %s\n", strout.c_str());

			// Just start a timer to introduce a delay to simulate a long asynchronous calculation 
			// after having read the response.
			// Delaying
			steady_timer client_timer(ioContextServer);
			print(PRI1, "mainflow_client: async_operation<void> st = commClient.start_timer(client_timer, 2000);\n");
			async_operation<void> st = commClient.start_timer(client_timer, 2000);
			print(PRI1, "mainflow_client: co_await st;\n");
			co_await st;

			// Closing
			print(PRI1, "mainflow_client: commClient.stop();\n");
			commClient.stop();
		}

		print(PRI1, "mainflow_client: co_return;\n");
		co_return;
	}

	oneway_task mainflow_reading_writing(spCommCore commCore)
	{
		// Reading
		print(PRI1, "mainflow_reading_writing: async_operation<std::string> sr = start_reading(clientSession);\n");
		async_operation<std::string> sr = commCore->start_reading();
		print(PRI1, "mainflow_reading_writing: std::string strout = co_await sr;\n");
		std::string strout = co_await sr;
		print(PRI1, "mainflow_reading_writing: received %s;\n", strout.c_str());

		// Communicate asynchronously with the server. Do not await its response.
		print(PRI1, "mainflow_reading_writing: mainflow_client();\n");
		(void)mainflow_client(strout);

		// Just start a timer to introduce a delay between reading and writing 
		// to simulate a long calculation.
		// Delaying
		boost::asio::steady_timer client_timer(m_IoContext);
		print(PRI1, "mainflow_reading_writing: async_operation<void> st = start_timer(client_timer, 300);\n");
		async_operation<void> st = commCore->start_timer(client_timer, 300);
		print(PRI1, "mainflow_reading_writing: co_await st;\n");
		co_await st;

		// Preparing output: convert the input string to uppercase
		std::string strtoecho = strout;
		for (auto& c : strtoecho) c = toupper(c);

		// Writing
		print(PRI1, "mainflow_reading_writing: async_operation<void> sw = start_writing(clientSession);\n");
		async_operation<void> sw = commCore->start_writing(strtoecho.c_str(), strtoecho.length() + 1);
		print(PRI1, "mainflow_reading_writing: co_await sw;\n");
		co_await sw;

		// Closing
		print(PRI1, "mainflow_reading_writing: clientSession->close();\n");
		commCore->stop();

		print(PRI1, "mainflow_reading_writing: co_return\n");
		co_return;
	}

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
			print(PRI1, "mainflow: mainflow_reading_writing(commCore);\n");
			(void)mainflow_reading_writing(commCore);
		}

		print(PRI1, "mainflow: co_return 0;\n");
		co_return 0;
	}
};

void runServer(
	ClientServerApp& server,
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

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

	ClientServerApp server1{ioContextServer, port_clientserver};

	std::thread t1{runServer, std::ref(server1), std::ref(ioContextServer)};

    t.join(); /// wait on stop signal

	print(PRI1, "main(): server1.stop()\n");
    server1.stop();
    t1.join();

	print(PRI1, "main(): return 0;\n");
	return 0;
}
