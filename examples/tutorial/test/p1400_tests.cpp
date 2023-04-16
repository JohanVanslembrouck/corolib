/**
 * @file p1400_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "gtest/gtest.h"

#include "../p1400.h"
#include "../eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;           // p1400.cpp

void completionflow1()
{
    print(PRI1, "completionflow1(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1402)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    print(PRI1, "p1402: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1402: completionflow1();\n");
    completionflow1();

    print(PRI1, "p1402: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1402: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

void completionflow2()
{
    print(PRI1, "completionflow2(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

TEST(TutorialTest, p1404)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    print(PRI1, "p1404: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1404: completionflow2();\n");
    completionflow2();

    print(PRI1, "p1404: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1404: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

#include "../eventqueuethr.h"

extern EventQueueThrFunctionVoidInt eventQueueThr;      // p1400.cpp

void completionflow3()
{
    print(PRI1, "completionflow3(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    runEventQueue(eventQueueThr, 2);
}

TEST(TutorialTest, p1405)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    print(PRI1, "p1405: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1405: completionflow3();\n");
    completionflow3();

    print(PRI1, "p1405: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1405: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

void completionflow4()
{
    print(PRI1, "completionflow4(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

TEST(TutorialTest, p1406)
{
    useMode = UseMode::USE_IMMEDIATE_COMPLETION;

    set_print_level(0x00);

    print(PRI1, "p1406: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1406: completionflow4();\n");
    completionflow4();

    print(PRI1, "p1406: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1406: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
