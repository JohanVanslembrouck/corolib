/** 
 * @file client4obs.cpp
 * @brief
 * Example of a client application.
 * This client application uses 1 CommClient object and 4 observer coroutines observer1 .. observer4
 * that each handle the result of the read operation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>
    
#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include <commclient.h>

#include "endpoints.h"

using namespace corolib;

class ClientApp : public CommClient
{
public:
    ClientApp(
        boost::asio::io_context & ioContext,
        boost::asio::ip::tcp::endpoint ep)
            : CommClient(ioContext, ep)
            , m_ioContext(ioContext)
    {
        print(PRI1, "ClientApp::ClientApp(...)\n");
    }

    /**
     * @brief observer1
     * @param op_read
     * @return always 1
     */
    async_task<int> observer1(async_operation<std::string>& op_read)
    {
        print(PRI1, "--- observer1: std::string strout = co_await op_read\n");
        std::string strout = co_await op_read;
        print(PRI1, "--- observer1: strout = %s", strout.c_str());
        co_return 1;
    }
    
    /**
     * @brief observer2
     * @param op_read
     * @return always 1
     */
    async_task<int> observer2(async_operation<std::string>& op_read)
    {
        print(PRI1, "--- observer2: std::string strout = co_await op_read\n");
        std::string strout = co_await op_read;
        print(PRI1, "--- observer2: strout = %s", strout.c_str());
        co_return 1;
    }
    
    /**
     * @brief observer3
     * @param op_read
     * @return always 1
     */
    async_task<int> observer3(async_operation<std::string>& op_read)
    {
        print(PRI1, "--- observer3: std::string strout = co_await op_read\n");
        std::string strout = co_await op_read;
        print(PRI1, "--- observer3: strout = %s", strout.c_str());
        co_return 1;
    }
    
    /**
     * @brief observer4
     * @param op_read
     * @return always 1
     */
    async_task<int> observer4(async_operation<std::string>& op_read)
    {
        print(PRI1, "--- observer4: std::string strout = co_await op_read\n");
        std::string strout = co_await op_read;
        print(PRI1, "--- observer4: strout = %s", strout.c_str());
        co_return 1;
    }

    /**
     * @brief
     * mainflow opens a connection to a server object, 
     * prepares a string and writes this string to the object,
     * reads the response and closes the connection. 
     * It repeats these actions 100 times.
     * @return always 0
     */
    async_task<int> mainflow()
    {
        print(PRI1, "mainflow: begin\n");
        int counter = 0;
        for (int i = 0; i < 100; i++)
        {
            print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", i);

            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
                steady_timer client_timer(m_ioContext);
                print(PRI1, "mainflow: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            // Connecting
            print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: co_await sc;\n");
            co_await sc;

            std::string str1 = "This is string ";
            str1 += std::to_string(counter++);
            str1 += " to echo\n";

            // Writing
            print(PRI1, "mainflow: async_operation<void> sw = start_writing(...);\n");
            async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
            print(PRI1, "mainflow: co_await sw;\n");
            co_await sw;

            // Reading
            // Construct an async_operation<std::string> awaitable
            print(PRI1, "mainflow: int index = get_free_index_ts();\n");
            int index = get_free_index_ts();
            print(PRI1, "mainflow: async_operation<std::string> sr{ this, index = %d, true };\n", index);
            async_operation<std::string> sr{ this, index, true };
            
            // Start the observer coroutines
            print(PRI1, "mainflow: async_task<int> sr1 = observer1(sr);\n");
            async_task<int> sr1 = observer1(sr);
            print(PRI1, "mainflow: async_task<int> sr2 = observer2(sr);\n");
            async_task<int> sr2 = observer2(sr);
            print(PRI1, "mainflow: async_task<int> sr3 = observer3(sr);\n");
            async_task<int> sr3 = observer3(sr);
            print(PRI1, "mainflow: async_task<int> sr4 = observer4(sr);\n");
            async_task<int> sr4 = observer4(sr);
            
            // Start reading
            print(PRI1, "mainflow: start_reading_impl(index = %d);\n", index);
            start_reading_impl(index);
            
            print(PRI1, "mainflow: when_all<async_operation<int>> war( { &sr1, &sr2, &sr3, &sr4 } );\n");
            when_all<async_task<int>> war( { &sr1, &sr2, &sr3, &sr4 } );
            // Wait until all observers have completed their task
            print(PRI1, "mainflow: before co_await war;\n");
            co_await war;
            print(PRI1, "mainflow: after co_await war;\n");

            // Just start a timer to introduce a delay to simulate a long asynchronous calculation 
            // after having read the response.
            // Delaying
            steady_timer client_timer(m_ioContext);
            print(PRI1, "mainflow: async_operation<void> st = start_timer(1000);\n");
            async_operation<void> st = start_timer(client_timer, 1000);
            print(PRI1, "mainflow: co_await st;\n");
            co_await st;

            // Closing
            print(PRI1, "mainflow: stop();\n");
            stop();
        }

        print(PRI1, "mainflow: co_return 0;\n");
        co_return 0;
    }

protected:
    boost::asio::io_context& m_ioContext;
};

int main()
{
    set_priority(0x01);

    boost::asio::io_context ioContext;

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c1(ioContext, ep1);

    print(PRI1, "main: async_task<int> si = c1.mainflow();\n");
    async_task<int> si = c1.mainflow();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
