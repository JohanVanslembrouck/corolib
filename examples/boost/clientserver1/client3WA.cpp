/** 
 * @file client3WA.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement CommClient.
 * This example uses 3 CommClient objects.
 *
 * It uses a when_all object that allows awaiting the completion of N asychronous operations.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

#include <commclient.h>

#include "endpoints.h"

using namespace corolib;

/**
 * @brief
 * mainflowWA0 uses 3 clients that each open a connection to a server (could be one and the same),
 * waits for the connection to each server to be established,
 * prepares 3 strings, one for every client/server,
 * writes the strings to the servers, waits for the 3 write operations to complete,
 * starts reading the response from each server, waits for the 3 read operations to complete
 * and then closes the connections.
 * mainflowWA0 performs the previous the previous actions 40 times.
 *
 * mainflowWA0 uses when_all to group all async_operation objects that have to be completed.
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c2 is the third client
 * @return always 0
 */
async_task<int> mainflowWA0(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflowWA0: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflowWA0: %d ------------------------------------------------------------------\n", i);

        // Connecting
        print(PRI1, "mainflowWA0: async_operation<void> sc1 = c1.start_connecting();\n");
        async_operation<void> sc1 = c1.start_connecting();
        print(PRI1, "mainflowWA0: async_operation<void> sc2 = c2.start_connecting();\n");
        async_operation<void> sc2 = c2.start_connecting();
        print(PRI1, "mainflowWA0: async_operation<void> sc3 = c3.start_connecting();\n");
        async_operation<void> sc3 = c3.start_connecting();

        print(PRI1, "mainflowWA0: when_all wac( { &sc1, &sc2, &sc3 } );\n");
        when_all wac({ &sc1, &sc2, &sc3 });
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

        print(PRI1, "mainflowWA0: when_all waw( { &sw1, &sw2, &sw3 } );\n");
        when_all waw({ &sw1, &sw2, &sw3 });
        print(PRI1, "mainflowWA0: co_await waw;\n");
        co_await waw;

        // Reading
        print(PRI1, "mainflowWA0: async_operation<std::string> sr1 = c1.start_reading();\n");
        async_operation<std::string> sr1 = c1.start_reading();
        print(PRI1, "mainflowWA0: async_operation<std::string> sr2 = c2.start_reading();\n");
        async_operation<std::string> sr2 = c2.start_reading();
        print(PRI1, "mainflowWA0: async_operation<std::string> sr3 = c3.start_reading();\n");
        async_operation<std::string> sr3 = c3.start_reading();

        print(PRI1, "mainflowWA0: when_all war( { &sr1, &sr2, &sr3 } ) ;\n");
        when_all war({ &sr1, &sr2, &sr3 });
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

/**
 * @brief
 * mainflowWA1 is a variant of mainflowWA0.
 * Instead of passing the 3 client objects as separate arguments as in mainflowWA0.
 * they are placed in a initializer list instead. 
 * This makes it possible to iterate over all clients to start an asynchronous operation.
 *
 * Also, mainflowWA1 avoids using separate names for each of the async_operation objects 
 * by using arrays of async_operation objects instead.
 * This means that the async_operation objects will not be immediately associated/initialized with the
 * operation they represent.
 * In the declaration of the arrays, the default constructor will be used
 * followed by the use of an assignment operator when the operation is started.
 *
 * @param clients is an initializer list with all clients
 * @return always 0
 */
async_task<int> mainflowWA1(std::initializer_list<CommClient*> clients)
{
    print(PRI1, "mainflowWA1: begin\n");

    const int nrClients = 3;     // TODO: clients.size();

    async_operation<void> asyncsc[nrClients];
    async_operation<void> asyncsw[nrClients];
    async_operation<std::string> asyncsr[nrClients];

    async_base* pasyncsc[nrClients];
    for (int i = 0; i < nrClients; ++i)
        pasyncsc[i] = &asyncsc[i];
    async_base* pasyncsw[nrClients];
    for (int i = 0; i < nrClients; ++i)
        pasyncsw[i] = &asyncsw[i];
    async_base* pasyncsr[nrClients];
    for (int i = 0; i < nrClients; ++i)
        pasyncsr[i] = &asyncsr[i];

    std::string results[nrClients];
    std::string str[nrClients];

    // Copy the clients from the initializer list:
    // we can iterate over this list only once.
    CommClient* _clients[nrClients];
    int cntr = 0;
    for (CommClient* cl : clients)
        _clients[cntr++] = cl;

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflowWA1: %d ------------------------------------------------------------------\n", i);

        print();
        print(PRI1, "mainflowWA1: connecting...\n");

        // Connecting
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA1: asyncsc[%d] = _clients[%d]->start_connecting();\n", j, j);
            asyncsc[j] = _clients[j]->start_connecting();
        }

        print(PRI1, "mainflowWA1: when_all wac(asyncsc, nrClients)\n");
        when_all wac(pasyncsc, nrClients);

        print(PRI1, "mainflowWA1: co_await wac;\n");
        co_await wac;

        print();
        print(PRI1, "mainflowWA1: writing...\n");

        // Writing
        for (int j = 0; j < nrClients; j++) {
            str[j] = "This is string ";
            str[j] += std::to_string(counter++);
            str[j] += " to echo\n";
            print(PRI1, "mainflowWA1: asyncsw[%d] = _clients[%d]->start_writing(...);\n", j, j);
            asyncsw[j] = _clients[j]->start_writing(str[j].c_str(), str[j].length() + 1);
        }

        print(PRI1, "mainflowWA1: when_all waw(asyncsw, 3)\n");
        when_all waw(pasyncsw, 3);

        print(PRI1, "mainflowWA1: co_await waw;\n");
        co_await waw;

        // Reading
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA1: asyncsr[%d] = _clients[%d]->start_reading();\n", j, j);
            asyncsr[j] = _clients[j]->start_reading();
        }

        print(PRI1, "mainflowWA1: when_all war(asyncsr, nrClients)\n");
        when_all war(pasyncsr, nrClients);

        print(PRI1, "mainflowWA1: co_await war;\n");
        co_await war;
        for (int j = 0; j < nrClients; j++)
            print(PRI1, "asyncsr[%d].get_result() = %s", j, asyncsr[j].get_result().c_str());

        // Closing
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA1: _clients[%d]->stop();\n", j);
            _clients[j]->stop();
        }

        print(PRI1, "mainflowWA1: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print(PRI1, "mainflowWA1: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflowWA2 is a variant of mainflowWA1
 * The async_operation arrays are declared inside the loop.
 *
 * @param clients is an initializer list with all clients
 * @return always 0
 */
async_task<int> mainflowWA2(std::initializer_list<CommClient*> clients)
{
    print(PRI1, "mainflowWA2: begin\n");
    const int nrClients = 3; // TODO: clients.size();
    
    // Copy the clients from the initializer list:
    // We can iterate over this list only once.
    CommClient* _clients[nrClients];
    int cntr = 0;
    for (CommClient* cl : clients)
        _clients[cntr++] = cl;

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflowWA2: %d ------------------------------------------------------------------\n", i);

        async_operation<void> asyncsc[nrClients];
        async_operation<void> asyncsw[nrClients];
        async_operation<std::string> asyncsr[nrClients];
        std::string str[nrClients];
        std::string results[nrClients];

        async_base* pasyncsc[nrClients];
        for (int i = 0; i < nrClients; ++i)
            pasyncsc[i] = &asyncsc[i];
        async_base* pasyncsw[nrClients];
        for (int i = 0; i < nrClients; ++i)
            pasyncsw[i] = &asyncsw[i];
        async_base* pasyncsr[nrClients];
        for (int i = 0; i < nrClients; ++i)
            pasyncsr[i] = &asyncsr[i];

        // Connecting
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA2: asyncsc[%d] = _clients[%d]->start_connecting();\n", j, j);
            asyncsc[j] = _clients[j]->start_connecting();
        }

        print(PRI1, "mainflowWA2: when_all wac(asyncsc, nrClients)\n");
        when_all wac(pasyncsc, nrClients);

        print(PRI1, "mainflowWA2: co_await wac;\n");
        co_await wac;

        // Writing
        for (int j = 0; j < nrClients; j++) {
            str[j] = "This is string ";
            str[j] += std::to_string(counter++);
            str[j] += " to echo\n";
            print(PRI1, "mainflowWA2: asyncsw[%d] = _clients[%d]->start_writing(...);\n", j, j);
            asyncsw[j] = _clients[j]->start_writing(str[j].c_str(), str[j].length() + 1);
        }

        print(PRI1, "mainflowWA2: when_all waw(asyncsw, nrClients)\n");
        when_all waw(pasyncsw, nrClients);

        print(PRI1, "mainflowWA2: co_await waw;\n");
        co_await waw;

        // Reading
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA2: asyncsr[%d] = _clients[%d]->start_reading();\n", j, j);
            asyncsr[j] = _clients[j]->start_reading();
        }

        print(PRI1, "mainflowWA2: when_all war(asyncsr, nrClients)\n");
        when_all war(pasyncsr, nrClients);

        print(PRI1, "mainflowWA2: co_await war;\n");
        co_await war;
        for (int j = 0; j < nrClients; j++)
            print(PRI1, "asyncsr[%d].get_result() = %s", j, asyncsr[j].get_result().c_str());

        // Closing
        for (int j = 0; j < nrClients; j++) {
            print(PRI1, "mainflowWA2: _clients[%d]->stop();\n", j);
            _clients[j]->stop();
        }

        print(PRI1, "mainflowWA2: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    print(PRI1, "mainflowWA2: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflowOneClient is a coroutine that is used an an auxiliary to mainflowWA3.
 *
 * @param c1 is the client object
 * @param instance allows distinguishing between several client objects while printing output
 * @param counter is used to compose a string that uses the value of the iteration counter of the calling coroutine
 * @return always 0
 *
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

    // Closing
    print(PRI1, "mainflowOneClient: %d: c1.stop();\n", instance );
    c1.stop();

    co_return 0;
}

/**
 * @brief
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflowWA3(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflowWA3: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflowWA3: %d ------------------------------------------------------------------\n", i);

        print(PRI1, "mainflowWA3: mainflowOneClient(c1, 0, counter++);\n");
        async_task<int> tc1 = mainflowOneClient(c1, 0, counter++);
        print(PRI1, "mainflowWA3: mainflowOneClient(c2, 1, counter++);\n");
        async_task<int> tc2 = mainflowOneClient(c2, 1, counter++);
        print(PRI1, "mainflowWA3: mainflowOneClient(c3, 2, counter++);\n");
        async_task<int> tc3 = mainflowOneClient(c3, 2, counter++);

        print(PRI1, "mainflowWA3: when_all wat({ &tc1, &tc2, &tc3 });\n");
        when_all wat({ &tc1, &tc2, &tc3 });

        print(PRI1, "mainflowWA3: co_await wat;\n");
        co_await wat;

        print(PRI1, "mainflowWA3: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    print(PRI1, "mainflowWA3: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
async_task<int> mainflowWA4(CommClient& c1, CommClient& c2, CommClient& c3)
{
    print(PRI1, "mainflowWA4: begin\n");

    int counter = 0;
    for (int i = 0; i < 60; i++)
    {
        print(PRI1, "mainflowWA4: %d ------------------------------------------------------------------\n", i);

        print(PRI1, "mainflowWA4: mainflowOneClient(c1, 0, counter++);\n");
        async_task<int> tc1 = mainflowOneClient(c1, 0, counter++);
        print(PRI1, "mainflowWA4: mainflowOneClient(c2, 1, counter++);\n");
        async_task<int> tc2 = mainflowOneClient(c2, 1, counter++);
        print(PRI1, "mainflowWA4: mainflowOneClient(c3, 2, counter++);\n");
        async_task<int> tc3 = mainflowOneClient(c3, 2, counter++);

        print(PRI1, "mainflowWA4: when_all wat(tc1, tc2, tc3);\n");
        when_all wat(tc1, tc2, tc3);

        print(PRI1, "mainflowWA4: co_await wat;\n");
        co_await wat;

        print(PRI1, "mainflowWA4: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    print(PRI1, "mainflowWA4: co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * mainflowX selects one of the 4 mainflowWA implementations (mainflowWA0 till mainflowWA3) to be used
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @param selected is the mainflowWA variant (defined above) to be used
 */
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
    case 4:
    {
        print(PRI1, "mainflowX: async_task<int> si4 = mainflowWA4(c1, c2, c3)\n");
        async_task<int> si4 = mainflowWA4(c1, c2, c3);
    }
    break;
    }
}

/**
 * @brief
 * mainflowAll executes all 4 mainflowWA implementations (mainflowWA0 till mainflowWA3) in sequence
 *
 * @param c1 is the first client
 * @param c2 is the second client
 * @param c3 is the third client
 * @return always 0
 */
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

    print(PRI1, "mainflowAll: async_task<int> si4 = mainflowWA4(c1, c2, c3} )\n");
    async_task<int> si4 = mainflowWA4(c1, c2, c3);
    print(PRI1, "mainflowAll: co_await si4;\n");
    co_await si4;

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
        if (selected < 0 || selected > 4)
        {
            print(PRI1, "main: selection must be in the range [0..4]\n");
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
