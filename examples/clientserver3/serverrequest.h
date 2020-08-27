/**
 * @file serverrequest.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _SERVERREQUEST_H_
#define _SERVERREQUEST_H_

#include "corolib/print.h"
#include "corolib/async_task.h"
#include "corolib/commserver.h"
#include "corolib/async_operation.h"
#include "corolib/oneway_task.h"

#include "reqresptypes.h"

using namespace corolib;

class ServerRequest
{
public:
	ServerRequest(spCommCore commClient, boost::asio::io_context& ioContext)
	: m_commClient(commClient)
	, m_ioContext(ioContext)
	{
		print(PRI1, "ServerRequest::ServerRequest(...)\n");
	}
	
	//oneway_task operation1(Req1&& req1)	// Use move to save copying?
	oneway_task operation1(Req1 req1)
	{
		print(PRI1, "operation1(Req1 req1)\n");
		
		// Delaying
		steady_timer client_timer(m_ioContext);
		print(PRI1, "operation1: async_operation st = m_commClient->start_timer(100);\n");
		async_operation st = m_commClient->start_timer(client_timer, 100);
		print(PRI1, "operation1: co_await st;\n");
		co_await st;
			
		// Preparing output
		std::string strout = "Resp1\n";
		
		// Writing
		print(PRI1, "operation1: async_operation sw = m_commClient->start_writing(...);\n");
		async_operation sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
		print(PRI1, "operation1: co_await sw;\n");
		co_await sw;
	}

	oneway_task operation2(Req2 req2)
	{
		print(PRI1, "operation2(Req2 req2)\n");
		
		// Delaying
		steady_timer client_timer(m_ioContext);
		print(PRI1, "operation2: async_operation st = m_commClient->start_timer(200);\n");
		async_operation st = m_commClient->start_timer(client_timer, 200);
		print(PRI1, "operation2: co_await st;\n");
		
		co_await st;
		// Preparing output
		std::string strout = "Resp2\n";

		// Writing
		print(PRI1, "operation2: async_operation sw = m_commClient->start_writing(...);\n");
		async_operation sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
		print(PRI1, "operation2: co_await sw;\n");
		co_await sw;
	}

	oneway_task operation3(Req3 req3)
	{
		print(PRI1, "operation3(Req3 req3)\n");
		
		// Delaying
		steady_timer client_timer(m_ioContext);
		print(PRI1, "operation3: async_operation st = m_commClient->start_timer(300);\n");
		async_operation st = m_commClient->start_timer(client_timer, 300);
		print(PRI1, "operation3: co_await st;\n");
		
		// Preparing output
		std::string strout = "Resp3\n";

		// Writing
		print(PRI1, "operation3: async_operation sw = m_commClient->start_writing(...);\n");
		async_operation sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
		print(PRI1, "operation3: co_await sw;\n");
		co_await sw;
	}

	oneway_task operation4(Req4 req4)
	{
		print(PRI4, "operation4(Req4 req4)\n");

		// Delaying
		steady_timer client_timer(m_ioContext);
		print(PRI1, "operation4: async_operation st = m_commClient->start_timer(400);\n");
		async_operation st = m_commClient->start_timer(client_timer, 400);
		print(PRI1, "operation4: co_await st;\n");
		
		// Preparing output
		std::string strout = "Resp4\n";

		// Writing
		print(PRI1, "operation4: async_operation sw = m_commClient->start_writing(...);\n");
		async_operation sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
		print(PRI1, "operation4: co_await sw;\n");
		co_await sw;
	}

private:
	spCommCore m_commClient;
	boost::asio::io_context& m_ioContext;
};

#endif
