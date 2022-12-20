/** 
 * @file client1.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a client application.
 * This example uses 1 CommClient object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/commclient.h>
#include <corolib/async_task.h>

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

    /**
     * @brief operation1 writes a request string onto the connection to the server 
     * and reads the response string.
     * Marshalling the Req1 content, and unmarshalling the response into Resp1 have been omitted.
     * @param Req1
     * @return async_task<Resp1>
     */
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

        std::string expect = "Resp1:params-go-here\n";
        print(PRI1, "operation1: expect = %s", expect.c_str());
        if (strout.compare(0, 5, expect, 0, 5) != 0)
        {
            print(PRI1, "operation1: expected response Resp1:params-go-here differs from actual response %s", strout.c_str());
        }
        co_return Resp1{};
    }
    
    /**
     * @brief operation2 writes a request string onto the connection to the server 
     * and reads the response string.
     * Marshalling the Req2 content, and unmarshalling the response into Resp2 have been omitted.
     * @param Req2
     * @return async_task<Resp2>
     */
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

        std::string expect = "Resp2:params-go-here\n";
        print(PRI1, "operation2: expect = %s", expect.c_str());
        if (strout.compare(0, 5, expect, 0, 5) != 0)
        {
            print(PRI1, "operation2: expected response Resp2:params-go-here differs from actual response %s", strout.c_str());
        }
        co_return Resp2{};
    }
    
    /**
     * @brief operation3 writes a request string onto the connection to the server 
     * and reads the response string.
     * Marshalling the Req3 content, and unmarshalling the response into Resp3 have been omitted.
     * @param Req3
     * @return async_task<Resp3>
     */
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

        std::string expect = "Resp3:params-go-here\n";
        print(PRI1, "operation3: expect = %s", expect.c_str());
        if (strout.compare(0, 5, expect, 0, 5) != 0)
        {
            print(PRI1, "operation3: expected response Resp3:params-go-here differs from actual response %s", strout.c_str());
        }
        co_return Resp3{};
    }
    
    /**
     * @brief operation4 writes a request string onto the connection to the server 
     * and reads the response string.
     * Marshalling the Req4 content, and unmarshalling the response into Resp4 have been omitted.
     * @param Req4
     * @return async_task<Resp4>
     */
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

        std::string expect = "Resp4:params-go-here\n";
        print(PRI1, "operation4: expect = %s", expect.c_str());
        if (strout.compare(0, 5, expect, 0, 5) != 0)
        {
            print(PRI1, "operation4: expected response Resp4:params-go-here differs from actual response %s", strout.c_str());
        }
        co_return Resp4{};
    }
    
    /**
     * @brief mainflow1 repeats the following actions 100 times:
     * it connects to the server
     * it invokes operation1, operation2, operation3 and operation4
     * it starts a timer of 1000 ms
     * it closes the connection to the server
     * @return async_task<int>
     */
    async_task<int> mainflow1()
    {
        print(PRI1, "mainflow1: begin\n");

        for (int i = 0; i < 100; i++)
        {
            print(PRI1, "mainflow1: i = %d ------------------------------------------------------------------\n", i);

            // Connecting
            print(PRI1, "mainflow1: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: co_await sc;\n");
            co_await sc;

            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
                steady_timer client_timer(ioContext);
                print(PRI1, "mainflow1: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            print(PRI1, "mainflow1: async_task<Resp1> res1 = operation1(Req1{})\n");
            async_task<Resp1> res1 = operation1(Req1{});
            print(PRI1, "mainflow1: Resp1 resp1 = co_await res1\n");
            Resp1 resp1 = co_await res1;
            (void)resp1;

            print(PRI1, "mainflow1: async_task<Resp2> res2 = operation2(Req2{})\n");
            async_task<Resp2> res2 = operation2(Req2{});
            print(PRI1, "mainflow1: Resp2 resp2 = co_await res2\n");
            Resp2 resp2 = co_await res2;
            (void)resp2;

            print(PRI1, "mainflow1: async_task<Resp3> res3 = operation3(Req3{})\n");
            async_task<Resp3> res3 = operation3(Req3{});
            print(PRI1, "mainflow1: Resp3 resp3 = co_await res3\n");
            Resp3 resp3 = co_await res3;
            (void)resp3;

            print(PRI1, "mainflow1: async_task<Resp4> res4 = operation4(Req4{})\n");
            async_task<Resp4> res4 = operation4(Req4{});
            print(PRI1, "mainflow1: Resp4 resp4 = co_await res4\n");
            Resp4 resp4 = co_await res4;
            (void)resp4;

            // Delaying
            steady_timer client_timer(ioContext);
            print(PRI1, "mainflow1: async_operation<void> st = start_timer(1000);\n");
            async_operation<void> st = start_timer(client_timer, 1000);
            print(PRI1, "mainflow1: co_await st;\n");
            co_await st;

            // Closing
            print(PRI1, "mainflow1: stop();\n");
            stop();
        }

        print(PRI1, "mainflow1: co_return;\n");
        co_return 0;
    }

    /**
  * @brief mainflow2 repeats the following actions 10 times:
  * it connects to the server
  * it invokes operation1, operation2, operation3 and operation4 10 times
  * it starts a timer of 1000 ms
  * it closes the connection to the server
  * @return async_task<int>
  */
    async_task<int> mainflow2()
    {
        print(PRI1, "mainflow2: begin\n");

        for (int i = 0; i < 10; i++)
        {
            print(PRI1, "mainflow2: i = %d ------------------------------------------------------------------\n", i);

            // Connecting
            print(PRI1, "mainflow2: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow2: co_await sc;\n");
            co_await sc;

            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
                steady_timer client_timer(ioContext);
                print(PRI1, "mainflow2: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            for (int j = 0; j < 10; j++)
            {
                print(PRI1, "mainflow2: i, j = %d, %d ------------------------------------------------------------------\n", i, j);

                print(PRI1, "mainflow2: async_task<Resp1> res1 = operation1(Req1{})\n");
                async_task<Resp1> res1 = operation1(Req1{});
                print(PRI1, "mainflow2: Resp1 resp1 = co_await res1\n");
                Resp1 resp1 = co_await res1;
                (void)resp1;

                print(PRI1, "mainflow2: async_task<Resp2> res2 = operation2(Req2{})\n");
                async_task<Resp2> res2 = operation2(Req2{});
                print(PRI1, "mainflow2: Resp2 resp2 = co_await res2\n");
                Resp2 resp2 = co_await res2;
                (void)resp2;

                print(PRI1, "mainflow2: async_task<Resp3> res3 = operation3(Req3{})\n");
                async_task<Resp3> res3 = operation3(Req3{});
                print(PRI1, "mainflow2: Resp3 resp3 = co_await res3\n");
                Resp3 resp3 = co_await res3;
                (void)resp3;

                print(PRI1, "mainflow2: async_task<Resp4> res4 = operation4(Req4{})\n");
                async_task<Resp4> res4 = operation4(Req4{});
                print(PRI1, "mainflow2: Resp4 resp4 = co_await res4\n");
                Resp4 resp4 = co_await res4;
                (void)resp4;

                // Delaying
                steady_timer client_timer(ioContext);
                print(PRI1, "mainflow2: async_operation<void> st = start_timer(1000);\n");
                async_operation<void> st = start_timer(client_timer, 1000);
                print(PRI1, "mainflow2: co_await st;\n");
                co_await st;
            }

            // Closing
            print(PRI1, "mainflow2: stop();\n");
            stop();
        }

        print(PRI1, "mainflow2: co_return;\n");
        co_return 0;
    }

    void mainflowX(int selected)
    {
        switch (selected) {
        case 0:
        {
            print(PRI1, "mainflowX: before async_task<int> si0= mainflow1();\n");
            async_task<int> si0 = mainflow1();
            print(PRI1, "mainflowX: after async_task<int> si0 = mainflow1();\n");
        }
        break;
        case 1:
        {
            print(PRI1, "mainflowX: before async_task<int> si1 = mainflow2();\n");
            async_task<int> si1 = mainflow2();
            print(PRI1, "mainflowX: after async_task<int> si1 = mainflow2();\n");
        }
        break;
        }
    }

    async_task<int> mainflowAll()
    {
        print(PRI1, "mainflowAll: before async_task<int> si0 = mainflow1();\n");
        async_task<int> si0 = mainflow1();
        print(PRI1, "mainflowAll: co_await si0;\n");
        co_await si0;

        print(PRI1, "mainflowAll: before async_task<int> si1 = mainflow2();\n");
        async_task<int> si1 = mainflow2();
        print(PRI1, "mainflowAll: co_await si1;\n");
        co_await si1;

        co_return 0;
    }
};

int main(int argc, char* argv[])
{
    set_priority(0x01);

    print(PRI1, "main: ClientApp c1(ioContext, ep);\n");
    ClientApp c1(ioContext, ep);

    if (argc == 2)
    {
        int selected = 0;
        selected = atoi(argv[1]);
        if (selected < 0 || selected > 1)
        {
            print(PRI1, "main: selection must be in the range [0..1]\n");
            return 0;
        }
        print(PRI1, "main: c1.mainflowX(selected);\n");
        c1.mainflowX(selected);
    }
    else
    {
        print(PRI1, "main: async_task<int> si = c1.mainflowAll();\n");
        async_task<int> si = c1.mainflowAll();
    }

    print(PRI1, "main: ioContext.run();\n");
    ioContext.run();

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
