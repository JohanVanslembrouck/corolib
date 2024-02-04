/**
 * @file p14XX_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/tracker.h>

#include "gtest/gtest.h"

 // -------------------------------------------------------------------------------------

#include "p1400.h"
#include "eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;
extern std::function<void(int)> eventHandler;

// ---------------------------------

void completionflow1400()
{
    // Begin manual event completion
    print(PRI1, "completionflow1400(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1400(): before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "completionflow1400(): after eventHandler(10);\n");

    print(PRI1, "completionflow1400(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1400(): before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "completionflow1400(): after eventHandler(10);\n");

    print(PRI1, "completionflow1400(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1400)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    print(PRI1, "p1400: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1400: completionflow1400();\n");
    completionflow1400();

    print(PRI1, "p1400: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1400: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1401()
{
    // Begin manual event completion
    print(PRI1, "completionflow1401(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    try {
        print(PRI1, "completionflow1401(): before eventHandler(-1);\n");
        eventHandler(-1);
        print(PRI1, "completionflow1401(): after eventHandler(-1);\n");
    }
    catch (...) {
        print(PRI1, "completionflow1401(): caught exception after eventHandler(-1);\n");
    }

    print(PRI1, "completionflow1401(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1401(): before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "completionflow1401(): after eventHandler(10);\n");

    print(PRI1, "completionflow1401(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1401)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    print(PRI1, "p1401(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1401(): completionflow1401();\n");
    completionflow1401();

    print(PRI1, "p1401(): int v = a.get_result(false);\n");
    int v = a.get_result(false);
    print(PRI1, "p1401(): v = %d\n", v);

    ASSERT_EQ(v, 17);
}

// ---------------------------------

void completionflow1402()
{
    print(PRI1, "completionflow1402(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1402)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    print(PRI1, "p1402: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1402: completionflow1402();\n");
    completionflow1402();

    print(PRI1, "p1402: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1402: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1404()
{
    print(PRI1, "completionflow1404(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1404)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    print(PRI1, "p1404: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1404: completionflow1404();\n");
    completionflow1404();

    print(PRI1, "p1404: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1404: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

#include "eventqueuethr.h"

extern EventQueueThrFunctionVoidInt eventQueueThr;      // p1400.cpp

// ---------------------------------

void completionflow1405()
{
    print(PRI1, "completionflow1405(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 2);
}

TEST(TutorialTest, p1405)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    print(PRI1, "p1405: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1405: completionflow1405();\n");
    completionflow1405();

    print(PRI1, "p1405: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1405: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1406()
{
    print(PRI1, "completionflow1406(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1406)
{
    useMode = UseMode::USE_IMMEDIATE_COMPLETION;

    set_print_level(0x00);

    print(PRI1, "p1406: async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "p1406: completionflow1406();\n");
    completionflow1406();

    print(PRI1, "p1406: int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1406: v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// -------------------------------------------------------------------------------------

#include "p1410.h"

void completionflow1410(Class1410& obj1410)
{
    // Begin manual event completion
    print(PRI1, "completionflow1410(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1410(): before object01.eventHandler(10);\n");
    obj1410.m_object01.eventHandler(10);
    print(PRI1, "completionflow1410(): after object01.eventHandler(10);\n");

    print(PRI1, "completionflow1410(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1410(): before object01.eventHandler(10);\n");
    obj1410.m_object01.eventHandler(10);
    print(PRI1, "completionflow1410(): after object01.eventHandler(10);\n");
    // End manual event completion
}

TEST(TutorialTest, p1410)
{
    set_print_level(0x00);

    Class01 object01;
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1410(): completionflow1410(obj1410);\n");
    completionflow1410(obj1410);

    print(PRI1, "p1410(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1411(Class1410& obj1410)
{
    // Begin manual event completion
    print(PRI1, "completionflow1411(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    try {
        print(PRI1, "completionflow1411(): before obj1410.m_object01.eventHandler(-1);\n");
        obj1410.m_object01.eventHandler(-1);
        print(PRI1, "completionflow1411(): after obj1410.m_object01.eventHandler(-1);\n");
    }
    catch (...) {
        print(PRI1, "completionflow1411: caught exception after obj1410.m_object01.eventHandler(-1);\n");
    }

    print(PRI1, "completionflow1411(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1411(): before obj1410.m_object01.eventHandler(10);\n");
    obj1410.m_object01.eventHandler(10);
    print(PRI1, "completionflow1411(): after obj1410.m_object01.eventHandler(10);\n");
    // End manual event completion
}

TEST(TutorialTest, p1411)
{
    set_print_level(0x00);

    Class01 object01;
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1411(): completionflow1411(obj1410);\n");
    completionflow1411(obj1410);

    print(PRI1, "p1411(): int v = a.get_result(false);\n");
    int v = a.get_result(false);
    print(PRI1, "p1411(): v = %d\n", v);

    ASSERT_EQ(v, 17);
}

// ---------------------------------

void completionflow1412()
{
    print(PRI1, "completionflow1412(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1412)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1412(): completionflow1412();\n");
    completionflow1412();

    print(PRI1, "p1412(): int v = awa.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1412(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1414()
{

}

TEST(TutorialTest, p1414)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD);
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1414(): completionflow1414();\n");
    completionflow1414();

    print(PRI1, "p1414(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1414(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1415()
{
    print(PRI1, "completionflow1415(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 2);
}

TEST(TutorialTest, p1415)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1415(): completionflow1415();\n");
    completionflow1415();

    print(PRI1, "p1415(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1415(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1416()
{

}

TEST(TutorialTest, p1416)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "p1416(): completionflow1416();\n");
    completionflow1416();

    print(PRI1, "p1416(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1416(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// -------------------------------------------------------------------------------------

#include "p1420.h"

void completionflow1420(Class1420& obj1420)
{
    // Begin manual event completion
    print(PRI1, "completionflow1420(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1420(): before obj1420.m_object02.eventHandler(10);\n");
    obj1420.m_object02.eventHandler(10);
    print(PRI1, "completionflow1420(): after obj1420.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1420(): before obj1420.m_object01.eventHandler(10);\n");
    obj1420.m_object01.eventHandler(10);
    print(PRI1, "completionflow1420(): after obj1420.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1420(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1420(): before obj1420.m_object01.eventHandler(10);\n");
    obj1420.m_object01.eventHandler(10);
    print(PRI1, "completionflow1420(): after obj1420.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1420(): before obj1420.m_object02.eventHandler(10);\n");
    obj1420.m_object02.eventHandler(10);
    print(PRI1, "completionflow1420(): after obj1420.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1420(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1420)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1420(): completionflow1420(obj1430);\n");
    completionflow1420(obj1420);

    print(PRI1, "p1420(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1420(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1421(Class1420& obj1420)
{
    // Begin manual event completion
    print(PRI1, "completionflow1421(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1421(): before object02.eventHandler(10);\n");
    obj1420.m_object02.eventHandler(10);
    print(PRI1, "completionflow1421(): after object02.eventHandler(10);\n");

    try {
        print(PRI1, "completionflow1421(): before object01.eventHandler(-1);\n");
        obj1420.m_object01.eventHandler(-1);
        print(PRI1, "completionflow1421(): after object01.eventHandler(-1);\n");
    }
    catch (...) {
        print(PRI1, "completionflow1421: caught exception after object01.eventHandler(-1);\n");
    }

    print(PRI1, "completionflow1421(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1421(): before object01.eventHandler(10);\n");
    obj1420.m_object01.eventHandler(10);
    print(PRI1, "completionflow1421(): after object01.eventHandler(10);\n");

    print(PRI1, "completionflow1421(): before object02.eventHandler(10);\n");
    obj1420.m_object02.eventHandler(10);
    print(PRI1, "completionflow1421(): after object02.eventHandler(10);\n");

    print(PRI1, "completionflow1421(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}


TEST(TutorialTest, p1421)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1421(): completionflow1421(obj1420);\n");
    completionflow1421(obj1420);

    print(PRI1, "p1421(): int v = a.get_result(false);\n");
    int v = a.get_result(false);
    print(PRI1, "p1421(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1422()
{
    print(PRI1, "completionflow1422(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1422)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1422(): completionflow1422();\n");
    completionflow1422();

    print(PRI1, "p1422(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1422(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1424()
{

}

TEST(TutorialTest, p1424)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1424(): completionflow1424();\n");
    completionflow1424();

    print(PRI1, "p1424(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1424(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1425()
{
    print(PRI1, "completionflow1425(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 4);
}

TEST(TutorialTest, p1425)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class01 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1425(): completionflow1425();\n");
    completionflow1425();

    print(PRI1, "p1425(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1425(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1426()
{

}

TEST(TutorialTest, p1426)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1426(): completionflow1426();\n");
    completionflow1426();

    print(PRI1, "p1426(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1426(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1428()
{
    print(PRI1, "completionflow1428(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1428)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1428(): completionflow1428();\n");
    completionflow1428();

    print(PRI1, "p1428(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1428(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// ---------------------------------

void completionflow1429()
{

}

TEST(TutorialTest, p1429)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1420 obj1420{ object01, object02 };
    async_task<int> a = obj1420.coroutine1();

    print(PRI1, "p1429(): completionflow1429();\n");
    completionflow1429();

    print(PRI1, "p1429(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1429(): v = %d\n", v);

    ASSERT_EQ(v, 47);
}

// -------------------------------------------------------------------------------------

#include "p1430.h"

void completionflow1430(Class1430& obj1430)
{
    // Begin manual event completion
    print(PRI1, "completionflow1430(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1430(): before obj1430.m_object01.eventHandler(10);\n");
    obj1430.m_object01.eventHandler(10);
    print(PRI1, "completionflow1430(): after obj1430.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1430(): before obj1430.m_object02.eventHandler(10);\n");
    obj1430.m_object02.eventHandler(10);
    print(PRI1, "completionflow1430(): after obj1430.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1430(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1430(): before obj1430.m_object02.eventHandler(10);\n");
    obj1430.m_object02.eventHandler(10);
    print(PRI2, "completionflow1430(): after obj1430.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1430(): before obj1430.m_object01.eventHandler(10);\n");
    obj1430.m_object01.eventHandler(10);
    print(PRI1, "completionflow1430(): after obj1430.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1430(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1430)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1430(): completionflow1430(obj1430);\n");
    completionflow1430(obj1430);

    print(PRI1, "p1430(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1430(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

#include "p1430a.h"

void completionflow1430a(Class1430a& obj1430a)
{
    // Begin manual event completion
    print(PRI1, "completionflow1430a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1430a(): before obj1430a.m_object01.eventHandler(10);\n");
    obj1430a.m_object01.eventHandler(10);
    print(PRI1, "completionflow1430a(): after obj1430a.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1430a(): before obj1430a.m_object02.eventHandler(10);\n");
    obj1430a.m_object02.eventHandler(10);
    print(PRI1, "completionflow1430a(): after obj1430a.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1430a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1430a(): before obj1430a.m_object02.eventHandler(10);\n");
    obj1430a.m_object02.eventHandler(10);
    print(PRI2, "completionflow1430a(): after obj1430a.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1430a(): before obj1430a.m_object01.eventHandler(10);\n");
    obj1430a.m_object01.eventHandler(10);
    print(PRI1, "completionflow1430a(): after obj1430a.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1430a(): std::this_thread::sleep_for(std::chrono::milliseconds(00));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1430a)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1430a obj1430a{ object01, object02 };
    async_task<int> a = obj1430a.coroutine1();

    print(PRI1, "p1430a(): completionflow1430a(obj1430);\n");
    completionflow1430a(obj1430a);

    print(PRI1, "p1430a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1430a(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1431()
{

}

TEST(TutorialTest, p1431)
{
    set_print_level(0x00);

    int v = 0;

    ASSERT_EQ(v, 0);
}

// ---------------------------------

void completionflow1432()
{
    print(PRI1, "completionflow1432(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1432)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1432(): completionflow1432();\n");
    completionflow1432();

    print(PRI1, "p1432(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1432(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------
#if 0
void completionflow1434()
{

}

TEST(TutorialTest, p1434)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1434(): completionflow1434();\n");
    completionflow1434();

    print(PRI1, "p1434(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1434(): v = %d\n", v);

    print(PRI1, "p1434(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 49);
}
#endif
// ---------------------------------

void completionflow1435()
{
    print(PRI1, "completionflow1435(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 4);
}

TEST(TutorialTest, p1435)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class01 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1435(): completionflow1435();\n");
    completionflow1435();

    print(PRI1, "p1435(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1435(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1436()
{

}

TEST(TutorialTest, p1436)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1436(): completionflow1436();\n");
    completionflow1436();

    print(PRI1, "p1436(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1436(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1438()
{
    print(PRI1, "completionflow1438(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1438)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1438(): completionflow1438();\n");
    completionflow1438();

    print(PRI1, "p1438(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1438(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1439()
{

}

TEST(TutorialTest, p1439)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1430 obj1430{ object01, object02 };
    async_task<int> a = obj1430.coroutine1();

    print(PRI1, "p1439(): completionflow1439();\n");
    completionflow1439();

    print(PRI1, "p1439(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1439(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// -------------------------------------------------------------------------------------

#include "p1440.h"

void completionflow1440(Class1440& obj1440)
{
    // Begin manual event completion
    print(PRI1, "completionflow1440(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1440(): before obj1440.m_object01.eventHandler(10);\n");
    obj1440.m_object01.eventHandler(10);
    print(PRI1, "completionflow1440(): after obj1440.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1440(): before obj1440.m_object02.eventHandler(10);\n");
    obj1440.m_object02.eventHandler(10);
    print(PRI1, "completionflow1440(): after obj1440.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1440(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1440(): before obj1440.m_object02.eventHandler(10);\n");
    obj1440.m_object02.eventHandler(10);
    print(PRI2, "completionflow1440(): after obj1440.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow1440(): before obj1440.m_object01.eventHandler(10);\n");
    obj1440.m_object01.eventHandler(10);
    print(PRI1, "completionflow1440(): after obj1440.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow1440(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1440)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1440(): completionflow1440(obj1440);\n");
    completionflow1440(obj1440);

    print(PRI1, "p1440(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1440(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1442()
{
    print(PRI1, "completionflow1442(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1442)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1442(): completionflow1442();\n");
    completionflow1442();

    print(PRI1, "p1442(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1442(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1444()
{
    print(PRI1, "completionflow1444(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1444)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1444(): completionflow1444();\n");
    completionflow1444();

    print(PRI1, "p1444(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1444(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1445()
{
    print(PRI1, "completionflow1445(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 4);
}
TEST(TutorialTest, p1445)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class01 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1445(): completionflow1445();\n");
    completionflow1445();

    print(PRI1, "p1445(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1445(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1446()
{
    print(PRI1, "completionflow1446(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1446)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1446(): completionflow1446();\n");
    completionflow1446();

    print(PRI1, "p1446(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1446(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1448()
{
    print(PRI1, "completionflow1448():  eventQueue.run();\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1448)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1448(): completionflow1448();\n");
    completionflow1448();

    print(PRI1, "p1448(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1448(): v = %d\n", v);

    ASSERT_EQ(v, 46);
}

// ---------------------------------

void completionflow1449()
{
    print(PRI1, "completionflow1449(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1449)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1440 obj1440{ object01, object02 };
    async_task<int> a = obj1440.coroutine1();

    print(PRI1, "p1449(): completionflow1449();\n");
    completionflow1449();

    print(PRI1, "p1449(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1449(): v = %d\n", v);

    ASSERT_EQ(v,46);
}

// -------------------------------------------------------------------------------------

#include "p1450.h"

void completionflow1450(Class1450& obj1450)
{
    // Begin manual event completion
    print(PRI1, "completionflow1450(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1450(): before obj1450.m_object01.eventHandler(10);\n");
        obj1450.m_object01.eventHandler(10);
        print(PRI1, "completionflow1450(): after obj1450.m_object01.eventHandler(10);\n");

        print(PRI1, "completionflow1450(): before obj1450.m_object02.eventHandler(10);\n");
        obj1450.m_object02.eventHandler(10);
        print(PRI1, "completionflow1450(): after obj1450.m_object02.eventHandler(10);\n");

        print(PRI1, "completionflow1450(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // End manual event completion
}

TEST(TutorialTest, p1450)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1450(): completionflow1450(obj1450);\n");
    completionflow1450(obj1450);

    print(PRI1, "p1450(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1450(): v = %d\n", v);

    print(PRI1, "p1450(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1452()
{
    print(PRI1, "completionflow1452(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1452)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1452(): completionflow1452();\n");
    completionflow1452();

    print(PRI1, "p1452(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1452(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1454()
{

}

TEST(TutorialTest, p1454)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1454(): completionflow1454();\n");
    completionflow1454();

    print(PRI1, "p1454(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1454(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1455()
{
    print(PRI1, "completionflow1455(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 8);
}

TEST(TutorialTest, p1455)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class01 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1455(): completionflow1455();\n");
    completionflow1455();

    print(PRI1, "p1455(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1455(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1456()
{

}

TEST(TutorialTest, p1456)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1456(): completionflow1456();\n");
    completionflow1456();

    print(PRI1, "p1456(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1456(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1458()
{
    print(PRI1, "completionflow1458(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1458)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1458(): completionflow1458();\n");
    completionflow1458();

    print(PRI1, "p1458(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1458(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// ---------------------------------

void completionflow1459()
{

}

TEST(TutorialTest, p1459)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1450 obj1450{ object01, object02 };
    async_task<int> a = obj1450.coroutine1();

    print(PRI1, "p1459(): completionflow1459();\n");
    completionflow1459();

    print(PRI1, "p1459(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1459(): v = %d\n", v);

    ASSERT_EQ(v, 89);
}

// -------------------------------------------------------------------------------------

#include "p1460.h"

void completionflow1460(Class1460& obj1460)
{
    // Begin manual event completion
    print(PRI1, "completionflow1460(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (int i = 0; i < 12; i++)
    {
        print(PRI1, "completionflow1460(): before obj1460.m_object01.eventHandler[%d](10);\n", i);
        obj1460.m_object01.eventHandler[i](10);
        print(PRI1, "completionflow1460(): after obj1460.m_object01.eventHandler[%d](10);\n", i);

        print(PRI1, "completionflow1460(): before obj1460.m_object02.eventHandler[%d](10);\n", i);
        obj1460.m_object02.eventHandler[i](10);
        print(PRI1, "completionflow1460(): after obj1460.m_object02.eventHandler[%d](10);\n", i);

        print(PRI1, "completionflow1460(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // End manual event completion
}

TEST(TutorialTest, p1460)
{
    set_print_level(0x00);

    Class02 object01;
    Class02 object02;
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1460(): completionflow1450(obj1460);\n");
    completionflow1460(obj1460);

    print(PRI1, "p1460(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1460(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// ---------------------------------

void completionflow1462()
{
    print(PRI1, "completionflow1462(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1462)
{
    set_print_level(0x00);

    Class02 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class02 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1462(): completionflow1462();\n");
    completionflow1462();

    print(PRI1, "p1462(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1462(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// ---------------------------------

void completionflow1464()
{

}

TEST(TutorialTest, p1464)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class02 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class02 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1464(): completionflow1464();\n");
    completionflow1464();

    print(PRI1, "p1464(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1464(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// ---------------------------------

void completionflow1465()
{
    print(PRI1, "completionflow1465(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    runEventQueue(eventQueueThr, 24);
}

TEST(TutorialTest, p1465)
{
    set_print_level(0x00);

    Class02 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class02 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1465(): completionflow1465();\n");
    completionflow1465();

    print(PRI1, "p1465(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1465(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// ---------------------------------

void completionflow1466()
{

}

TEST(TutorialTest, p1466)
{
    set_print_level(0x00);

    Class02 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class02 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1466(): completionflow1466();\n");
    completionflow1466();

    print(PRI1, "p1466(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1466(): v = %d\n", v);


    ASSERT_EQ(v, 249);
}

// ---------------------------------

void completionflow1468()
{
    print(PRI1, "completionflow1468(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

TEST(TutorialTest, p1468)
{
    set_print_level(0x00);

    Class02 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class02 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1468(): completionflow1468();\n");
    completionflow1468();

    print(PRI1, "p1468(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1468(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// ---------------------------------
void completionflow1469()
{

}

TEST(TutorialTest, p1469)
{
    set_print_level(0x00);

    Semaphore sema{ 1 };
    Class02 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class02 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1460 obj1460{ object01, object02 };
    async_task<int> a = obj1460.coroutine1();

    print(PRI1, "p1469(): completionflow1469();\n");
    completionflow1469();

    print(PRI1, "p1469(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1469(): v = %d\n", v);

    ASSERT_EQ(v, 249);
}

// -------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
