/**
 * @file serverrequest2.h
 * @brief server-side implementation of the 4 operations
 * The operations return async_task<int> instead of oneway_task as in serverrequest.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _SERVERREQUEST2_H_
#define _SERVERREQUEST2_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>

#include <commserver.h>

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
    
    /**
     * @brief operation1 writes a response to the client after a delay of 100 ms.
     * operation1 does not take the content of req1 into account and it prepares the response string
     * without marshalling it from a Resp1 object.
     * @param req1
     * @return async_task<int> returning 0
     */
    async_task<int> operation1(Req1 req1)
    {
        print(PRI1, "operation1(Req1 req1)\n");
        
        // Delaying
        steady_timer client_timer(m_ioContext);
        print(PRI1, "operation1: async_operation<void> st = m_commClient->start_timer(100);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 100);
        print(PRI1, "operation1: co_await st;\n");
        co_await st;
            
        // Preparing output
        std::string strout = "Resp1:params-go-here\n";
        
        // Writing
        print(PRI1, "operation1: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation1: co_await sw;\n");
        co_await sw;
        
        co_return 0;
    }

    /**
     * @brief operation2 writes a response to the client after a delay of 200 ms.
     * operation2 does not take the content of req2 into account and it prepares the response string
     * without marshalling it from a Resp2 object.
     * @param req2
     * @return async_task<int> returning 0
     */
    async_task<int> operation2(Req2 req2)
    {
        print(PRI1, "operation2(Req2 req2)\n");
        
        // Delaying
        steady_timer client_timer(m_ioContext);
        print(PRI1, "operation2: async_operation<void> st = m_commClient->start_timer(200);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 200);
        print(PRI1, "operation2: co_await st;\n");
        
        co_await st;
        // Preparing output
        std::string strout = "Resp2:params-go-here\n";

        // Writing
        print(PRI1, "operation2: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation2: co_await sw;\n");
        co_await sw;

        co_return 0;
    }

    /**
     * @brief operation3 writes a response to the client after a delay of 300 ms.
     * operation3 does not take the content of req3 into account and it prepares the response string
     * without marshalling it from a Resp3 object.
     * @param req3
     * @return async_task<int> returning 0
     */
    async_task<int> operation3(Req3 req3)
    {
        print(PRI1, "operation3(Req3 req3)\n");
        
        // Delaying
        steady_timer client_timer(m_ioContext);
        print(PRI1, "operation3: async_operation<void> st = m_commClient->start_timer(300);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 300);
        print(PRI1, "operation3: co_await st;\n");
        
        // Preparing output
        std::string strout = "Resp3:params-go-here\n";

        // Writing
        print(PRI1, "operation3: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation3: co_await sw;\n");
        co_await sw;

        co_return 0;
    }

    /**
     * @brief operation4 writes a response to the client after a delay of 400 ms.
     * operation4 does not take the content of req4 into account and it prepares the response string
     * without marshalling it from a Resp4 object.
     * @param req4
     * @return async_task<int> returning 0
     */
    async_task<int> operation4(Req4 req4)
    {
        print(PRI4, "operation4(Req4 req4)\n");

        // Delaying
        steady_timer client_timer(m_ioContext);
        print(PRI1, "operation4: async_operation<void> st = m_commClient->start_timer(400);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 400);
        print(PRI1, "operation4: co_await st;\n");
        
        // Preparing output
        std::string strout = "Resp4:params-go-here\n";

        // Writing
        print(PRI1, "operation4: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation4: co_await sw;\n");
        co_await sw;

        co_return 0;
    }

private:
    spCommCore m_commClient;
    boost::asio::io_context& m_ioContext;
};

#endif
