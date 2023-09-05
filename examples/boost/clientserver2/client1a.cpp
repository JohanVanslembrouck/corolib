/** 
 * @file client1a.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a client application.
 * This example uses 1 CommClient object.
 *
 * See README.md for further information.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/when_any.h>

#include <commclient.h>

#include "endpoints.h"

using namespace corolib;

int secondtimeout = 200;

const int NR_ITERATIONS_MAINLOOP1 = 100;
const int NR_OUTER_ITERATIONS_MAINLOOP2 = 10;
const int NR_INNER_ITERATIONS_MAINLOOP2 = 10;

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
      * @brief performAction writes a request to the server and uses a timed read to wait for the response
      * @param i is the action number that is also used to calculate how long to wait for the response
      * A different timeout is used for even and odd numbers.
      * @param last is used to inform the server if this is the last request or not. In case of a last request,
      * the server (and client) will close the connection;
      * @return async_task<void>
     */
    async_task<void> performAction(int i, bool last)
    {
        print(PRI1, "performAction: begin\n");

        // Prepare the request
        std::string str1;
        str1 = last ? "LAST: " : "MORE: ";
        str1 += "This is string ";
        str1 += std::to_string(i);
        str1 += " to echo\n";

        // Writing
        print(PRI1, "performAction: async_operation<void> sw = start_writing(...);\n");
        async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
        print(PRI1, "performAction: co_await sw;\n");
        co_await sw;

        // The server waits 1000 ms before sending the response.
        // Alternating, wait 2000 ms or only 200 ms (default value of secondtimeout).
        print(PRI1, "performAction: async_task<server_response_t<std::string>> pA = start_reading_timed(%d);\n", (i % 2) ? 2000 : secondtimeout);
        async_task<server_response_t<std::string>> pA = start_reading_timed((i % 2) ? 2000 : secondtimeout);
        print(PRI1, "performAction: server_response_t<std::string> res = co_await pA;\n");
        server_response_t<std::string> res = co_await pA;
        if (res.response_status == resp_arrived)
            print(PRI1, "performAction: res.response.c_str() = %s\n", res.response.c_str());
        else
            print(PRI1, "performAction: response did not arrive in time;\n");

        print(PRI1, "performAction: co_return\n");
        co_return;
    }

    enum response_status_t
    {
        resp_not_yet_in,
        resp_arrived,
        resp_timeout
    };
    
    template<typename TYPE>
    struct server_response_t
    {
        response_status_t response_status;
        TYPE response;
    };

    /**
     * @brief acknowledgeAction writes an acknowledgement "ACK\n" onto the connection to the server.
     *
     * @return async_task<int> with return value 0
     */
    async_task<int> acknowledgeAction()
    {
        // Prepare the ACK request
        std::string str1 = "ACK\n";

        // Writing
        print(PRI1, "acknowledgeAction: async_operation<void> sw = start_writing(...);\n");
        async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
        print(PRI1, "acknowledgeAction: co_await sw;\n");
        co_await sw;

        print(PRI1, "acknowledgeAction: co_return 0;\n");
        co_return 0;
    }
    
    /**
     * @brief start_reading_timed starts an interaction with the server and waits for the response.
     * The behavior of start_reading_timed in this file is the same as that 
     * of start_reading_timed in client1.cpp.
     * The major difference is the use of a loop of maximum 2 iterations.
     * The content of the loop combines the two parts of start_reading_timed in client1.cpp
     * co_wait war is only used once.
     * The reader is referred to the description of start_reading_timed in client1.cpp.
     *
     * @param the timeout to use
     * @return async_task<int> with return value 0
     */
    async_task<server_response_t<std::string>> start_reading_timed(int timeout)
    {
        server_response_t<std::string> resp;
        resp.response_status = resp_not_yet_in;
        
        // Reading
        print(PRI1, "start_reading_timed: async_operation<std::string> sr = start_reading();\n");
        async_operation<std::string> sr = start_reading();

        // Timing
        steady_timer client_timer(ioContext);
        print(PRI1, "start_reading_timed: async_operation<void> st = start_timer(client_timer, timeout);\n");
        async_operation<void> st = start_timer(client_timer, timeout);
        
        //print(PRI1, "start_reading_timed: when_anyT<async_operation_base> war( { &sr, &st } ) ;\n");
        //when_anyT<async_operation_base> war({ &sr, &st });
        print(PRI1, "start_reading_timed: when_any war( { &sr, &st } ) ;\n");
        when_any war({ &sr, &st });

        bool done = false;
        for (int j = 0; j < 2 && !done; j++)
        {
            print(PRI1, "start_reading_timed: int i = co_await war;\n");
            int i = co_await war;

            print(PRI1, "start_reading_timed: (%d, %d)\n", j, i);
            switch (i)
            {
            case 0:    // Reply has arrived, stop the timer and print the result
            {
                print(PRI1, "start_reading_timed: i = %d: reply has arrived, stop the timer\n", i);

                print(PRI1, "start_reading_timed: client_timer.cancel();\n");
                client_timer.cancel();
                
                if (j == 0)
                {
                    print(PRI1, "start_reading_timed: i = %d: reply has arrived in time\n", i);
                    
                    resp.response_status = resp_arrived;
                    resp.response = sr.get_result();
                    
                    done = true;

                    // Send acknowledgement to server
                    print(PRI1, "start_reading_timed: async_task<int> ackAction = acknowledgeAction();\n");
                    async_task<int> ackAction = acknowledgeAction();
                    print(PRI1, "start_reading_timed: co_await ackAction;\n");
                    co_await ackAction;
                }
                else
                {
                    print(PRI1, "start_reading_timed: i = %d, answer arrived after STOP was sent\n", i);
                    
                    resp.response_status = resp_arrived;
                    resp.response = sr.get_result();
                }
            }
            break;
            case 1: 
            {
                if (j == 0)        // Timer has expired a first time, send a cancel request
                {
                    print(PRI1, "start_reading_timed: i = %d: timer has expired a first time, send a cancel request\n", i);

                    // Prepare the STOP request
                    std::string str1 = "STOP\n";

                    // Writing
                    print(PRI1, "start_reading_timed: async_operation<void> sw2 = start_writing(...);\n");
                    async_operation<void> sw2 = start_writing(str1.c_str(), str1.length() + 1);
                    print(PRI1, "start_reading_timed: co_await sw2;\n");
                    co_await sw2;

                    // Start a timer of 500 ms
                    // Timing
                    print(PRI1, "start_reading_timed: st = start_timer(500);\n");
                    st = start_timer(client_timer, 500);
                }
                else
                {
                    print(PRI1, "start_reading_timed: i = %d: timer has expired a second time\n", i);
                    
                    resp.response_status = resp_timeout;
                    resp.response = "";
                }
            }
            break;
            default:
                print(PRI1, "start_reading_timed: i = %d: should not occur\n", i);
            }
        }

        print(PRI1, "start_reading_timed: co_return 0;\n");
        co_return resp;
    }
    
    /**
     * @brief mainflow1 repeats the following actions 100 times:
     * 1) it connects to the server
     * 2) it calls start_writing to write a string to the server
     * 3) it calls start_reading_timed (see above) to read the response from the server
     * 4) it starts a timer of 100 ms
     * 5) it closes the connection
     *
     * @return async_task<int> with return value 0
     */
    async_task<int> mainflow1()
    {
        print(PRI1, "mainflow1: begin\n");

        for (int i = 0; i < NR_ITERATIONS_MAINLOOP1; i++)
        {
            print(PRI1, "mainflow1: %d ------------------------------------------------------------------\n", i);

            // Connecting
            print(PRI1, "mainflow1: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow1: co_await sc;\n");
            co_await sc;
            
            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
                steady_timer client_timer(ioContext);
                print(PRI1, "mainflow1: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            print(PRI1, "mainflow1: co_await performAction(%d, true);\n", i);
            co_await performAction(i, true);

            // Prepare the request
            std::string str1 = "This is string ";
            str1 += std::to_string(i);
            str1 += " to echo\n";

            // Wait some time between iterations: timer has to be called before calling stop().
            // Timing
            steady_timer client_timer(ioContext);
            print(PRI1, "mainflow1: co_await start_timer(100);\n");
            co_await start_timer(client_timer, 100);

            // Closing
            print(PRI1, "mainflow1: stop();\n");
            stop();
        }

        print(PRI1, "mainflow1: co_return 0;\n");
        co_return 0;
    }

    async_task<int> mainflow2()
    {
        print(PRI1, "mainflow2: begin\n");

        for (int i = 0; i < NR_OUTER_ITERATIONS_MAINLOOP2; i++)
        {
            print(PRI1, "mainflow2: %d ------------------------------------------------------------------\n", i);

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

            for (int j = 0; j < NR_INNER_ITERATIONS_MAINLOOP2; j++)
            {
                print(PRI1, "mainflow: co_await performAction(%d, %d);\n", j, j == NR_INNER_ITERATIONS_MAINLOOP2-1);
                co_await performAction(j, j == NR_INNER_ITERATIONS_MAINLOOP2-1);
            }

            // Wait some time between iterations: call timer before calling stop().
            // Timing
            steady_timer client_timer(ioContext);
            print(PRI1, "mainflow2: co_await start_timer(100);\n");
            co_await start_timer(client_timer, 100);

            // Closing
            print(PRI1, "mainflow2: stop();\n");
            stop();
        }

        print(PRI1, "mainflow2: co_return 0;\n");
        co_return 0;
    }

    void mainflowX(int selected)
    {
        switch (selected) {
        case 0:
        {
            print(PRI1, "mainflowX: async_task<int> si0 = mainflow();\n");
            async_task<int> si0 = mainflow1();
        }
        break;
        case 1:
        {
            print(PRI1, "mainflowX: async_task<int> si1 = mainflow2()\n");
            async_task<int> si1 = mainflow2();
        }
        break;
        }
    }

    async_task<int> mainflowAll()
    {
        print(PRI1, "mainflowAll: async_task<int> si0 = mainflow1();\n");
        async_task<int> si0 = mainflow1();
        print(PRI1, "mainflowAll: co_await si0;\n");
        co_await si0;

        print(PRI1, "mainflowAll: async_task<int> si1 = mainflow2()\n");
        async_task<int> si1 = mainflow2();
        print(PRI1, "mainflowAll: co_await si1;\n");
        co_await si1;

        print(PRI1, "mainflowAll: co_return 0;\n");
        co_return 0;
    }
};

int main(int argc, char* argv[])
{
    set_priority(0x01);

    print(PRI1, "main: ClientApp c1(ioContext, ep);\n");
    ClientApp c1(ioContext, ep);

    if (argc == 3)
        secondtimeout = atoi(argv[3]);

    if (argc >= 2)
    {
        int selected = 0;
        selected = atoi(argv[1]);
        if (selected < 0 || selected > 3)
        {
            print(PRI1, "main: selection must be in the range [0..3]\n");
            return 0;
        }
        print(PRI1, "main: mainflowX(cselected);\n");
        c1.mainflowX(selected);
    }
    else
    {
        print(PRI1, "main: async_task<int> si = mainflowAll();\n");
        async_task<int> si = c1.mainflowAll();
    }

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
