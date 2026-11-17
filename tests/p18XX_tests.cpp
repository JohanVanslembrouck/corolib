/**
 * @file p18XX_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "gtest/gtest.h"

 // -------------------------------------------------------------------------------------

#include "p1800.h"
#include "eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;
extern std::function<void(int)> eventHandler;

// ---------------------------------

void completionflow1800(Class1800& obj)
{
    print(PRI1, "completionflow1800()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1800(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));

        print(PRI1, "completionflow1800(): before obj.op.set_result_and_complete(10);\n");
        obj.op.set_result_and_complete(10);
        print(PRI1, "completionflow1800(): after obj.op.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow1800(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    // Make coroutine1 co_return
    print(PRI1, "completionflow1800(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1800(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1800)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    Class1800 obj;

    print(PRI1, "p1800(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1800(): completionflow1800(obj);\n");
    completionflow1800(obj);

    print(PRI1, "p1800(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1800(): v = %d\n", v);
    
    ASSERT_EQ(v, 41);
}

// ---------------------------------

void completionflow1802(Class1800& obj)
{
    print(PRI1, "completionflow1802()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1802(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1802(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1802(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1802(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1802)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    Class1800 obj;

    print(PRI1, "p1802(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1802(): completionflow1802(obj);\n");
    completionflow1802(obj);

    print(PRI1, "p1802(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1802(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// ---------------------------------

void completionflow1804(Class1800& obj)
{
    print(PRI1, "completionflow1804()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1804(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1804(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1804(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1804(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1804(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1804)
{
    useMode = UseMode::USE_THREAD;

    Class1800 obj;

    print(PRI1, "p1804(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1804(): completionflow1804(obj);\n");
    completionflow1804(obj);

    print(PRI1, "p1804(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1804(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// ---------------------------------

#include "eventqueuethr.h"

extern EventQueueThrFunctionVoidInt eventQueueThr;

// ---------------------------------

void completionflow1805(Class1800& obj)
{
    print(PRI1, "completionflow1805()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1805(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1805(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1805(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    print(PRI1, "completionflow1805(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1805(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1805(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1805)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    Class1800 obj;

    print(PRI1, "p1805(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1805(): completionflow1805(obj);\n");
    completionflow1805(obj);

    print(PRI1, "p1805(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1805(): v = %d\n", v);
   
    ASSERT_EQ(v, 41);
}

// -------------------------------------------------------------------------------------

#include "p1810.h"

void completionflow1810(Class1810& obj)
{
    print(PRI1, "completionflow1810()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1810(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));

        print(PRI1, "completionflow1810(): before obj.op.set_result_and_complete(10);\n");
        obj.op.set_result_and_complete(10);
        print(PRI1, "completionflow1810(): after obj.op.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow1810(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    // Make coroutine1 co_return
    print(PRI1, "completionflow1810(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1810(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1810)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    Class1810 obj;

    print(PRI1, "p1810(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1810(): completionflow1810(obj);\n");
    completionflow1810(obj);

    print(PRI1, "p1810(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1810(): v = %d\n", v);

    ASSERT_EQ(v, 83);
}

// ---------------------------------

void completionflow1812(Class1810& obj)
{
    print(PRI1, "completionflow1812()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1812(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1812(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1812(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1812(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1812)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    Class1810 obj;

    print(PRI1, "p1812(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1812(): completionflow1812(obj);\n");
    completionflow1812(obj);

    print(PRI1, "p1812(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1812(): v = %d\n", v);

    ASSERT_EQ(v, 83);
}

// ---------------------------------

void completionflow1814(Class1810& obj)
{
    print(PRI1, "completionflow1814()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1814(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1814(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1814(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1814(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1814(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1814)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    //delay = 10;

    Class1810 obj;

    print(PRI1, "p1814(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1814(): completionflow1814(obj);\n");
    completionflow1814(obj);

    print(PRI1, "p1814(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1814(): v = %d\n", v);

    ASSERT_EQ(v, 83);
}

// ---------------------------------

void completionflow1815(Class1810& obj)
{
    print(PRI1, "completionflow1815()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1815(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1815(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1815(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    print(PRI1, "completionflow1815(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1815(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow1815(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion
}

TEST(TutorialTest, p1815)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    Class1810 obj;

    print(PRI1, "p1815(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1815(): completionflow1815(obj);\n");
    completionflow1815(obj);

    print(PRI1, "p1815(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1815(): v = %d\n", v);

    ASSERT_EQ(v, 83);
}

// -------------------------------------------------------------------------------------

#include "p1820.h"

void completionflow1820(Class1820& obj)
{
    print(PRI1, "completionflow1820()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1820(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1820(): before obj.op.set_result_and_complete(10);\n");
        obj.op.set_result_and_complete(10);
        print(PRI1, "completionflow1820(): after obj.op.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow1820(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Make coroutine1 co_return
    print(PRI1, "completionflow1820(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1820(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1820)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    Class1820 obj;

    print(PRI1, "p1820(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1820(): completionflow1820(obj);\n");
    completionflow1820(obj);

    print(PRI1, "p1820(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1820(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// ---------------------------------

void completionflow1822(Class1820& obj)
{
    print(PRI1, "completionflow1822()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1822(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1822(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1822(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1822(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1822)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    Class1820 obj;

    print(PRI1, "p1822(): async_task<int> a = coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1822(): completionflow1822();\n");
    completionflow1822(obj);

    print(PRI1, "p1822(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1822(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// ---------------------------------

void completionflow1824(Class1820& obj)
{
    print(PRI1, "completionflow1824()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1824(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1824(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1824(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1824(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1824(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1824)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    Class1820 obj;

    print(PRI1, "p1824(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1824(): completionflow1824(obj);\n");
    completionflow1824(obj);

    print(PRI1, "p1824(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1824(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// ---------------------------------

void completionflow1825(Class1820& obj)
{
    print(PRI1, "completionflow1825()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1825(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1825(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1825(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1825(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1825(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1825)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    Class1820 obj;

    print(PRI1, "p1825(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1825(): completionflow1825(obj);\n");
    completionflow1825(obj);

    print(PRI1, "p1825(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1825(): v = %d\n", v);

    ASSERT_EQ(v, 41);
}

// -------------------------------------------------------------------------------------

#include "p1830.h"

void completionflow1830(Class1830& obj)
{
    print(PRI1, "completionflow1830()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1830(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1830(): before obj.op.set_result_and_complete(10);\n");
        obj.op.set_result_and_complete(10);
        print(PRI1, "completionflow1830(): after obj.op.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow1830(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Make coroutine1 co_return
    print(PRI1, "completionflow1830(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1830(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1830)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    Class1830 obj;

    print(PRI1, "p1830(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1830(): completionflow1830(obj);\n");
    completionflow1830(obj);

    print(PRI1, "p1830(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1830(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

void completionflow1832(Class1830& obj)
{
    print(PRI1, "completionflow1832()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1832(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1832(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1832(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1832(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1832)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    Class1830 obj;

    print(PRI1, "p1832(): async_task<int> a = coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1832(): completionflow1832();\n");
    completionflow1832(obj);

    print(PRI1, "p1832(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1832(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

void completionflow1834(Class1830& obj)
{
    print(PRI1, "completionflow1834()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1834(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1834(): obj.start_operation_impl(op);\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1834(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1834(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1834(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1834)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    Class1830 obj;

    print(PRI1, "p1834(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1834(): completionflow1834(obj);\n");
    completionflow1834(obj);

    print(PRI1, "p1834(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1834(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

void completionflow1835(Class1830& obj)
{
    print(PRI1, "completionflow1835()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1835(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1835(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1835(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1835(): before obj.op.set_result_and_complete(std::nullopt);\n");
    obj.op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1835(): after obj.op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

TEST(TutorialTest, p1835)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    Class1830 obj;

    print(PRI1, "p1835(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1835(): completionflow1835(obj);\n");
    completionflow1835(obj);

    print(PRI1, "p1835(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1835(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// -------------------------------------------------------------------------------------

#include "p1840.h"
#include "corolib/when_all.h"

async_task<void> completionflow1840(Class1840& obj, async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow1840()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1840(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1840(): before obj.op1.set_result_and_complete(10);\n");
        obj.op1.set_result_and_complete(10);
        print(PRI1, "completionflow1840(): after obj.op1.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow1840(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Make coroutine1 co_return
    print(PRI1, "completionflow1840(): before obj.op1.set_result_and_complete(std::nullopt);\n");
    obj.op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1840(): after obj.op1.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow1840(): when_all wa(a1, a2, a3);\n");
    when_all wa(a1, a2, a3);
    print(PRI1, "completionflow1840(): co_await wa;\n");
    co_await wa;
}

TEST(TutorialTest, p1840)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    Class1840 obj;

    print(PRI1, "p1840(): async_task<int> a1 = obj.coroutine1();\n");
    async_task<int> a1 = obj.coroutine1();
    print(PRI1, "p1840(): async_task<int> a2 = obj.coroutine2();\n");
    async_task<int> a2 = obj.coroutine2();
    print(PRI1, "p1840(): async_task<int> a3 = obj.coroutine3();\n");
    async_task<int> a3 = obj.coroutine3();

    print(PRI1, "p1840(): completionflow1840(obj, a1, a2, a3);\n");
    completionflow1840(obj, a1, a2, a3);

    print(PRI1, "p1840(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
    int v = a1.get_result() + a2.get_result() + a3.get_result();
    print(PRI1, "p1840(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

async_task<void> completionflow1842(Class1840& obj, async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow1842()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1842(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1842(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1842(): before obj.op1.set_result_and_complete(std::nullopt);\n");
    obj.op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1842(): after obj.op1.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow1842(): when_all wa(a1, a2, a3);\n");
    when_all wa(a1, a2, a3);
    print(PRI1, "completionflow1842(): co_await wa;\n");
    co_await wa;
}

TEST(TutorialTest, p1842)
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_print_level(0x00);

    Class1840 obj;

    print(PRI1, "p1842(): async_task<int> a1 = obj.coroutine1();\n");
    async_task<int> a1 = obj.coroutine1();
    print(PRI1, "p1842(): async_task<int> a2 = obj.coroutine2();\n");
    async_task<int> a2 = obj.coroutine2();
    print(PRI1, "p1842(): async_task<int> a3 = obj.coroutine3();\n");
    async_task<int> a3 = obj.coroutine3();

    print(PRI1, "p1842(): completionflow1842(obj, a1, a2, a3);\n");
    completionflow1842(obj, a1, a2, a3);

    print(PRI1, "p1842(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
    int v = a1.get_result() + a2.get_result() + a3.get_result();
    print(PRI1, "p1842(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

async_task<void> completionflow1844(Class1840& obj, async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow1844()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1844(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1844(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1844(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1844(): before obj.op1.set_result_and_complete(std::nullopt);\n");
    obj.op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1844(): after obj.op1.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow1844(): when_all wa(a1, a2, a3);\n");
    when_all wa(a1, a2, a3);
    print(PRI1, "completionflow1844(): co_await wa;\n");
    co_await wa;
}

TEST(TutorialTest, p1844)
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x00);

    Class1840 obj;

    print(PRI1, "p1844(): async_task<int> a1 = obj.coroutine1();\n");
    async_task<int> a1 = obj.coroutine1();
    print(PRI1, "p1844(): async_task<int> a2 = obj.coroutine2();\n");
    async_task<int> a2 = obj.coroutine2();
    print(PRI1, "p1844(): async_task<int> a3 = obj.coroutine3();\n");
    async_task<int> a3 = obj.coroutine3();

    print(PRI1, "p1844(): completionflow1844(obj, a1, a2, a3);\n");
    completionflow1844(obj, a1, a2, a3);

    print(PRI1, "p1844(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
    int v = a1.get_result() + a2.get_result() + a3.get_result();
    print(PRI1, "p1844(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// ---------------------------------

async_task<void> completionflow1845(Class1840& obj, async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow1845()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow1845(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow1845(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow1845(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow1845(): before obj.op1.set_result_and_complete(std::nullopt);\n");
    obj.op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow1845(): after obj.op1.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow1845(): when_all wa(a1, a2, a3);\n");
    when_all wa(a1, a2, a3);
    print(PRI1, "completionflow1845(): co_await wa;\n");
    co_await wa;
}

TEST(TutorialTest, p1845)
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x00);

    Class1840 obj;

    print(PRI1, "p1845(): async_task<int> a1 = obj.coroutine1();\n");
    async_task<int> a1 = obj.coroutine1();
    print(PRI1, "p1845(): async_task<int> a2 = obj.coroutine2();\n");
    async_task<int> a2 = obj.coroutine2();
    print(PRI1, "p1845(): async_task<int> a3 = obj.coroutine3();\n");
    async_task<int> a3 = obj.coroutine3();

    print(PRI1, "p1845(): completionflow1845(obj, a1, a2, a3);\n");
    completionflow1845(obj, a1, a2, a3);

    print(PRI1, "p1845(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
    int v = a1.get_result() + a2.get_result() + a3.get_result();
    print(PRI1, "p1845(): v = %d\n", v);

    ASSERT_EQ(v, 123);
}

// -------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
