/** 
 * @file client3.cpp
 * @brief
 * Example of a client application.
 * This client application uses 3 CommClient objects.
 * Apart from async_operation and async_task, this example
 * also uses oneway_task and auto_reset_event.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/oneway_task.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_operation.h>

#include <commclient.h>

#include "endpoints.h"

using namespace corolib;

boost::asio::io_context ioContext;

/**
 * @brief
 * mainflowOneClient opens a connection to a server, 
 * prepares a string and writes this string to the server,
 * reads the response and closes the connection.
 *
 * mainflowOneClient has oneway_task as return type.
 * Therefore the calling coroutine cannot use co_await to wait for mainflowOneClient to return.
 * This example shows an alternative way using an auto_reset_event parameter.
 * As a last statement, mainflowOneClient calls resume() on the auto_reset_event object
 * to resume the coroutine that co_waits the object.
 *
 * @param c1 is the client object
 * @param are (short for auto_reset_event) allows mainflowOneClient to resume the coroutine that co_awaits the are object
 * @param instance allows distinguishing between several client objects while printing output
 * @param counter is used to compose a string that uses the value of the iteration counter of the calling coroutine
 * @return oneway_task (cannot be co_waited upon)
 */
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

    // Delaying
    steady_timer client_timer(ioContext);
    print(PRI1, "mainflowOneClient: async_operation<void> st = c1.start_timer(100);\n");
    async_operation<void> st = c1.start_timer(client_timer, 100);
    print(PRI1, "mainflowOneClient: co_await st;\n");
    co_await st;

    // Closing
    print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance);
    c1.stop();

    // Resume the coroutine that co_awaits are.
    are.resume();
}

/**
 * @brief
 * mainflowOneClient opens a connection to a server, 
 * prepares a string and writes this string to the server,
 * reads the response and closes the connection.
 * This implementation returns an aync_task object, which is the normal way in case
 * another coroutine wants to await the completion of mainflowOneClient.
 * @param c1 is the client object
 * @param allows to distinguish between several client objects while printing output
 * @param counter is used to compose a string that uses the value of the iteration counter of the calling coroutine
 * @return always 0
 */
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

    // Delaying
    steady_timer client_timer(ioContext);
    print(PRI1, "mainflowOneClient: async_operation<void> st = c1.start_timer(100);\n");
    async_operation<void> st = c1.start_timer(client_timer, 100);
    print(PRI1, "mainflowOneClient: co_await st;\n");
    co_await st;

    // Closing
    print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance);
    c1.stop();

    co_return 0;
}

/**
 * @brief
 * mainflow0 uses 3 clients that each open a connection to a server (could be one and the same),
 * waits for the connection to each server to be established,
 * prepares 3 strings, one for every client/server,
 * writes the strings to the servers, waits for the 3 write operations to complete,
 * starts reading the response from each server, waits for the 3 read operations to complete
 * and then closes the connections.
 * mainflow0 performs the previous the previous actions 40 times.
 *
 * The code can be shorter using when_all instead of using 3 co_await statements.
 * This will be illustrated in other examples.
 * 
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c2 is the third client
 * @return always 0
 */
async_task<int> mainflow0(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow0: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
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

        // The order of waiting is not important.
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

        steady_timer client_timer(ioContext);
        print(PRI1, "mainflow0: async_operation<void> st = c1.start_timer(100);\n");
        async_operation<void> st = c1.start_timer(client_timer, 100);
        print(PRI1, "mainflow0: co_await st;\n");
        co_await st;

        print(PRI1, "mainflow0: c1.stop();\n");
        c1.stop();
        print(PRI1, "mainflow0: c2.stop();\n");
        c2.stop();
        print(PRI1, "mainflow0: c3.stop();\n");
        c3.stop();
    }

    print(PRI1, "mainflow0: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow1 shows how to use mainflowOneClient that returns a oneway_task object
 * and takes an auto_reset_event object as argument.
 * mainflow1 co_awaits each of the auto_reset_event objects to be "informed"
 * of the completion of the mainflowOneClient coroutines.
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflow1(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow1: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflow1: %d ------------------------------------------------------------------\n", i);

        auto_reset_event are1, are2, are3;
    
        print(PRI1, "mainflow1: mainflowOneClient(c1, are1, 0, counter++);\n");
        (void) mainflowOneClient(c1, are1, 0, counter++);
        print(PRI1, "mainflow1: mainflowOneClient(c2, are2, 1, counter++);\n");
        (void) mainflowOneClient(c2, are2, 1, counter++);
        print(PRI1, "mainflow1: mainflowOneClient(c3, are3, 2, counter++);\n");
        (void) mainflowOneClient(c3, are3, 2,  counter++);

        print(PRI1, "mainflow1: co_await are1;\n");
        co_await are1;
        print(PRI1, "mainflow1: co_await are2;\n");
        co_await are2;
        print(PRI1, "mainflow1: co_await are3;\n");
        co_await are3;
        
        print(PRI1, "mainflow1: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    print(PRI1, "mainflow1: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow2 uses the mainflowOneClient coroutine that returns 
 * an async_task object. This variant is preferred over mainflow1 above. 
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflow2(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow2: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
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

        print(PRI1, "mainflow2: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    print(PRI1, "mainflow2: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow3 uses 3 clients, but avoids using separate names for each of the async_operation objects 
 * by using arrays of async_operation objects instead.
 * This means that the async_operation objects will not be immediately associated/initialized with the
 * operation they represent.
 * In the declaration of the arrays, the default constructor will be used
 * followed by the use of an assignment operator when the operation is started.
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflow3(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow3: begin\n");

    const int nrClients = 3;
    int counter = 0;
    for (int i = 0; i < 60; i++)
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

        // Use a loop to co_await the completion
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

        print(PRI1, "mainflow3: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    print(PRI1, "mainflow3: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow4 is a variant of mainflow3, but it declares the arrays outside the for loop.
 * This means that the async_operation objects will not go out of scope at the end of each iteration,
 * but that they will be re-used for every new iteration.
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the second client
 * @return always 0
 */
async_task<int> mainflow4(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflow4: begin\n");

    const int nrClients = 3;
    async_operation<void> asyncsc[nrClients];
    async_operation<void> asyncsw[nrClients];
    async_operation<std::string> asyncsr[nrClients];
    std::string results[nrClients];
    int counter = 0;

    for (int i = 0; i < 60; i++)
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

        print(PRI1, "mainflow4: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    print(PRI1, "mainflow4: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow5 is a variant of mainflow3.
 * Instead of passing the 3 client objects as separate arguments as in mainflow3,
 * they are placed in a initializer list instead. 
 * This makes it possible to iterate over all clients to start an asynchronous operation
 * as well as over the async_operation objects to co_await their completion.
 * @param clients is an initializer list with all clients
 * @return always 0
 *
 */
async_task<int> mainflow5(std::initializer_list<CommClient*> client_il)
{
    print(PRI1, "mainflow5: begin\n");

    const int nrClients = 3;    // TODO: clients.size()
    int counter = 0;

    // Copy the clients from the initializer list:
    // we can iterate over this list only once.
    CommClient* clients[nrClients];
    int cntr = 0;
    for (CommClient* cl : client_il)
        clients[cntr++] = cl;

    for (int i = 0; i < 60; i++)
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

        print(PRI1, "mainflow5: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    print(PRI1, "mainflow5: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflow6 is a variant of mainflow4.
 * Instead of passing the 3 client objects as separate arguments as in mainflow4,
 * they are placed in an initializer list instead. 
 * This makes it possible to iterate over all clients to start an asynchronous operation
 * as well as over the async_operation objects to co_await their completion.
 * @param clients is an iniatializer list will all clients
 * @return always 0
 */
async_task<int> mainflow6(std::initializer_list<CommClient*> client_il)
{
    print(PRI1, "mainflow6: begin\n");

    const int nrClients = 3;
    async_operation<void> asyncsc[nrClients];
    async_operation<void> asyncsw[nrClients];
    async_operation<std::string> asyncsr[nrClients];
    std::string results[nrClients];

    int counter = 0;

    // Copy the clients from the initializer list:
    // we can iterate over this list only once.
    CommClient* clients[nrClients];
    int cntr = 0;
    for (CommClient* cl : client_il)
        clients[cntr++] = cl;

    for (int i = 0; i < 60; i++)
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

        print(PRI1, "mainflow6: std::this_thread::sleep_for(std::chrono::milliseconds(100))\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    print(PRI1, "mainflow6: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflowX selects one of the 7 mainflow implementations (mainflow0 till mainflow6) to be used
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @param selected is the mainflow variant (defined above) to be used
 */
async_task<int> mainflowX(CommClient& c1, CommClient& c2, CommClient& c3, int selected)
{
    switch (selected) {
    case 0:
    {
        print(PRI1, "mainflowX: async_task<int> si0 = mainflow0(c1, c2, c3);\n");
        async_task<int> si0 = mainflow0(c1, c2, c3);
        print(PRI1, "mainflowX: co_await si0;\n");
        co_await si0;
    }
    break;
    case 1:
    {
        print(PRI1, "mainflowX: async_task<int> si1 = mainflow1(c1, c2, c3);\n");
        async_task<int> si1 = mainflow1(c1, c2, c3);
        print(PRI1, "mainflowX: co_await si1;\n");
        co_await si1;
    }
    break;
    case 2:
    {
        print(PRI1, "mainflowX: async_task<int> si2 = mainflow2(c1, c2, c3);\n");
        async_task<int> si2 = mainflow2(c1, c2, c3);
        print(PRI1, "mainflowX: co_await si2;\n");
        co_await si2;
    }
    break;
    case 3:
    {
        print(PRI1, "mainflowX: async_task<int> si3 = mainflow3(c1, c2, c3);\n");
        async_task<int> si3 = mainflow3(c1, c2, c3);
        print(PRI1, "mainflowX: co_await si3;\n");
        co_await si3;
    }
    break;
    case 4:
    {
        print(PRI1, "mainflowX: async_task<int> si4 = mainflow4(c1, c2, c3);\n");
        async_task<int> si4 = mainflow4(c1, c2, c3);
        print(PRI1, "mainflowX: co_await si4;\n");
        co_await si4;
    }
    break;
    case 5:
    {
        print(PRI1, "mainflowX: async_task<int> si5 = mainflow5({&c1, &c2, &c3})\n");
        async_task<int> si5 = mainflow5({ &c1, &c2, &c3 });
        print(PRI1, "mainflowX: co_await si5;\n");
        co_await si5;
    }
    break;
    case 6:
    {
        print(PRI1, "mainflowX: async_task<int> si6 = mainflow6({&c1, &c2, &c3})\n");
        async_task<int> si6 = mainflow6({ &c1, &c2, &c3 });
        print(PRI1, "mainflowX: co_await si6;\n");
        co_await si6;
    }
    break;
    }
    co_return 0;
}

/**
 * @brief
 * mainflowAll executes all 7 mainflow implementations (mainflow0 till mainflow6) in sequence
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflowAll(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflowAll: before async_task<int> si0 = mainflow0(c1, c2, c3);\n");
    async_task<int> si0 = mainflow0(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si0;\n");
    co_await si0;
    
    print(PRI1, "mainflowAll: before async_task<int> si1 = mainflow1(c1, c2, c3);\n");
    async_task<int> si1 = mainflow1(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si1;\n");
    co_await si1;

    print(PRI1, "mainflowAll: before async_task<int> si2 = mainflow2(c1, c2, c3);\n");
    async_task<int> si2 = mainflow2(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si2;\n");
    co_await si2;
    
    print(PRI1, "mainflowAll: before async_task<int> si3 = mainflow3(c1, c2, c3);\n");
    async_task<int> si3 = mainflow3(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si3;\n");
    co_await si3;
    
    print(PRI1, "mainflowAll: before async_task<int> si4 = mainflow4(c1, c2, c3);\n");
    async_task<int> si4 = mainflow4(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si4;\n");
    co_await si4;
    
    print(PRI1, "mainflowAll: before async_task<int> si5 = mainflow5({&c1, &c2, &c3})\n");
    async_task<int> si5 = mainflow5({ &c1, &c2, &c3 });
    print(PRI1, "mainflowAll: co_await si5;\n");
    co_await si5;
    
    print(PRI1, "mainflowAll: before async_task<int> si6 = mainflow6({&c1, &c2, &c3})\n");
    async_task<int> si6 = mainflow6({ &c1, &c2, &c3 });
    print(PRI1, "mainflowAll: co_await si6;\n");
    co_await si6;
    
    print(PRI1, "mainflowAll: co_return 0;\n");
    co_return 0;
}

int main(int argc, char* argv[])
{
    set_priority(0x01);

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
        if (selected < 0 || selected > 6)
        {
            print(PRI1, "main: selection must be in the range [0..6]\n");
            return 0;
        }
        print(PRI1, "main: mainflowX(c1, c2, c3, selected);\n");
        async_task<int> si = mainflowX(c1, c2, c3, selected);

        // Keep mainflowX in the same scope as ioContext.run() to
        // ensure that all promise_type objects and final_awaiter objects
        // are released.
        print(PRI1, "main: before ioContext.run();\n");
        ioContext.run();
        print(PRI1, "main: after ioContext.run();\n");
    }
    else
    {
        print(PRI1, "main: async_task<int> si = mainflowAll(c1, c2, c3);\n");
        async_task<int> si = mainflowAll(c1, c2, c3);

        // Keep mainflowAll in the same scope as ioContext.run() to
        // ensure that all promise_type objects and final_awaiter objects
        // are released.
        print(PRI1, "main: before ioContext.run();\n");
        ioContext.run();
        print(PRI1, "main: after ioContext.run();\n");
    }

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
