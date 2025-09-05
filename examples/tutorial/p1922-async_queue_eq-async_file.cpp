/**
 * @file p1922-async_queue_eq-async_file.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <thread>

#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/async_queue_eq.h>
#include <corolib/print.h>

using namespace corolib;

#include "async_file.h"

EventQueueFunctionVoidVoid eventQueue;

const int QUEUESIZE = 4;
async_queue_eq<std::string, EventQueueFunctionVoidVoid, QUEUESIZE> queue(eventQueue, false);

async_file file(eventQueue);

async_task<void> testfile()
{
    int ret = -1;
    print(PRI1, "testfile: ret = co_await file.start_write(...)\n", ret);
    ret = co_await file.start_write("str");
    print(PRI1, "testfile: ret = %d\n", ret);
    print(PRI1, "testfile: co_return;\n");
    co_return;
}

class Class1922 : public CommService
{
public:
    Class1922(EventQueueFunctionVoidVoid& eventQueue)
        : m_eventQueue(eventQueue)
    {

    }

    async_operation<void> dummy_op()
    {
        int index = get_free_index();
        async_operation<void> ret{ this, index };

        auto f = [this, index]() {
            print(PRI5, "async_create handler: begin\n");
            completionHandler_v(index);
            print(PRI5, "async_create handler: end\n");
        };
        m_eventQueue.push(std::move(f));
        return ret;
    }

    async_task<void> filewriter()
    {
        while (true)
        {
            int ret = -1;
            print(PRI1, "filewriter: str = co_await queue.pop();\n");
            std::string str = co_await queue.pop();
            if (str.size() == 0)
                break;
            print(PRI1, "filewriter: str = %s\n", str.c_str());

            print(PRI1, "filewriter: co_await file.start_open(\"alarms.txt\"); \n");
            ret = co_await file.start_open("alarms.txt");
            (void)ret;

            print(PRI1, "filewriter: co_await object01.start_write(str);\n");
            ret = co_await file.start_write(str);
            (void)ret;

            print(PRI1, "filewriter: co_await object01.start_close();\n");
            ret = co_await file.start_close();
            (void)ret;
        }
        print(PRI1, "filewriter: co_return;\n");
        co_return;
    }

    async_task<void> filereader()
    {
        int ret = -1;
        print(PRI1, "filereader:  co_await file.start_create(\"alarms.txt\"); \n");
        ret = co_await file.start_create("alarms.txt");
        (void)ret;

        print(PRI1, "filereader: co_await file.start_open(\"alarms.txt\"); \n");
        ret = co_await file.start_open("alarms.txt");
        (void)ret;

        async_file::read_result result = co_await file.start_read();
        if (result.result == 0)
            print(PRI1, "filereader: content = \n%s\n", result.content.c_str());

        print(PRI1, "filereader: co_await object01.start_close();\n");
        ret = co_await file.start_close();
        (void)ret;

        print(PRI1, "co_return: end\n");
        co_return;
    }

    async_task<void> producer(int nr_operations)
    {
        print(PRI1, "producer: begin\n");
        for (int counter = 0; counter < nr_operations; counter++)
        {
            print(PRI1, "producer: co_await dummy_op();\n");
            co_await dummy_op();

            std::string str = "This is string " + std::to_string(counter) + " to save";
            print(PRI1, "producer: before co_await queue.push(%s)\n", str.c_str());
            co_await queue.push(str);
            print(PRI1, "producer: after co_await queue.push(%s)\n", str.c_str());
        }
        // Signal end of producer
        print(PRI1, "producer: co_await queue.push(\"\");\n");
        co_await queue.push("");
        print(PRI1, "producer: end\n");
        co_return;
    }

    async_task<void> mainflow(int nr_operations)
    {
        print(PRI1, "mainflow: async_task<void> tf = testfile();\n");
        async_task<void> tf = testfile();
        print(PRI1, "mainflow: co_await tf;\n");
        co_await tf;

        print(PRI1, "filewriter:  co_await file.start_create(\"alarms.txt\"); \n");
        int ret = co_await file.start_create("alarms.txt");
        (void)ret;

        print(PRI1, "mainflow: async_task<void> fw = filewriter();\n");
        async_task<void> fw = filewriter();
        print(PRI1, "mainflow: async_task<void> pr = producer();\n");
        async_task<void> pr = producer(nr_operations);
        print(PRI1, "mainflow: co_await when_all(fw, pr);\n");
        co_await when_all(fw, pr);

        print(PRI1, "mainflow: async_task<void> fr = filereader();\n");
        async_task<void> fr = filereader();
        print(PRI1, "mainflow: co_await fr;\n");
        co_await fr;

        print(PRI1, "filereader:  co_await file.start_remove(\"alarms.txt\"); \n");
        co_await file.start_remove("alarms.txt");

        print(PRI1, "mainflow: co_return;\n");
        co_return;
    }

private:
    EventQueueFunctionVoidVoid& m_eventQueue;
};

void test01(Class1922& obj1922, int nr_operations, int sleeptime)
{
    print(PRI1, "test01: async_task<void> mf = mainflow();\n");
    async_task<void> mf = obj1922.mainflow(nr_operations);
    print(PRI1, "test01: runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue, sleeptime);
    print(PRI1, "test01: mf.wait();\n");
    mf.wait();
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib
                                  // Use 0x11 to follow the flow in async_file.h
                                  // Use 0x13 to follow the flow in async_file.h and in corolib

    print(PRI1, "main: begin\n");

    Class1922 obj1922(eventQueue);

    print(PRI1, "main: test01(obj1922, 10, 10);\n");
    test01(obj1922, 10, 10);
    print(PRI1, "main: test01(obj1922, 50, 10);\n");
    test01(obj1922, 50, 10);

    set_print_level(0x00);

    printf("main: test01(obj1922, 100, 0);\n");
    test01(obj1922, 100, 0);
    printf("main: test01(obj1922, 10000, 0);\n");
    test01(obj1922, 10000, 0);
    printf("main: test01(obj1922, 100000, 0);\n");
    test01(obj1922, 100000, 0);
    printf("main: test01(obj1922, 1000000, 0);\n");
    test01(obj1922, 1000000, 0);
    printf("main: test01(obj1922, 2000000, 0);\n");
    test01(obj1922, 2000000, 0);

    print(PRI1, "main: return 0;\n");

    set_print_level(0x01);
	return 0;
}
