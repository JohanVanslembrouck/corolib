/**
 * @file p17XX_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "gtest/gtest.h"

 // -------------------------------------------------------------------------------------

#include "p1700.h"
#include "eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;
extern std::function<void(int)> eventHandler;

#include "eventqueuethr.h"

extern EventQueueThrFunctionVoidInt eventQueueThr;

// ---------------------------------

void completionflow1700()
{
    // Begin manual event completion
    print(PRI1, "completionflow1700(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1700(): 1. before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "completionflow1700(): 1. after eventHandler(10);\n");

    print(PRI1, "completionflow1700(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1700(): 2. before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "completionflow1700(): 2. after eventHandler(10);\n");

    print(PRI1, "completionflow1700(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1700)
{
    useMode = UseMode::USE_NONE;

    set_print_level(0x00);

    print(PRI1, "p1700(): async_ltask<int> a = coroutine1();\n");
    async_ltask<int> a = coroutine1();
	print(PRI1, "p1700(): a.start();\n");
	a.start();

    print(PRI1, "p1700(): completionflow1700();\n");
    completionflow1700();

    print(PRI1, "p1700(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1700(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// ---------------------------------

void completionflow1706()
{
    print(PRI1, "completionflow1706(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TEST(TutorialTest, p1706)
{
    useMode = UseMode::USE_IMMEDIATE_COMPLETION;

    set_print_level(0x00);

    print(PRI1, "p1706(): async_ltask<int> a = coroutine1();\n");
    async_ltask<int> a = coroutine1();
	print(PRI1, "p1706(): a.start();\n");
    a.start();
		
    print(PRI1, "p1706(): completionflow1706();\n");
    completionflow1706();

    print(PRI1, "p1706(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1706(): v = %d\n", v);

    ASSERT_EQ(v, 27);
}

// -------------------------------------------------------------------------------------

#include "p1730.h"

void completionflow1730(Class1730& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow1730(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1730(): 1a. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow1730(): 1a. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow1730(): 1b. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow1730(): 1b. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1730(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1730(): 2a. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI2, "completionflow1730(): 2a. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1730(): 2b. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow1730(): 2b. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow1730(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1730)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	print(PRI1, "p1730(): a.start();\n");
    a.start();

    print(PRI1, "p1730(): completionflow1730(obj);\n");
    completionflow1730(obj);

    print(PRI1, "p1730(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1730(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

#include "p1730a.h"

void completionflow1730a(Class1730a& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow1730a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1730a(): 1a. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow1730a(): 1a. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow1730a(): 1b. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow1730a(): 1b. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1730a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1730a(): 2a. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI2, "completionflow1730a(): 2a. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1730a(): 2b. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow1730a(): 2b. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow1730a(): std::this_thread::sleep_for(std::chrono::milliseconds(00));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // End manual event completion
}

TEST(TutorialTest, p1730a)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1730a obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
    a.start();

    print(PRI1, "p1730a(): completionflow1730a(obj);\n");
    completionflow1730a(obj);

    print(PRI1, "p1730a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1730a(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1731(Class1730& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow1731(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow1731(): 1a. before object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow1731(): 1a. after object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1731(): 1b. before obj.m_object01.runEventHandler(-1);\n");
    obj.m_object01.runEventHandler(-1);
    print(PRI1, "completionflow1731(): 1b. after obj.m_object01.runEventHandler(-1);\n");

    print(PRI1, "completionflow1731(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow1731(): 2a. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow1731(): 2a. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow1731(): 2b. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow1731(): 2b. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow1731(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
    // End manual event completion
}

TEST(TutorialTest, p1731)
{
    set_print_level(0x00);

    Class01 object01;
    Class01 object02;
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	a.start();
	
    print(PRI1, "p1731(): completionflow1731(obj);\n");
    completionflow1731(obj);

    print(PRI1, "p1731(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1731(): v = %d\n", v);

    ASSERT_EQ(v, 39);
}

// ---------------------------------

void completionflow1732()
{
    print(PRI1, "completionflow1732(): runEventQueue(eventQueue, %d);\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);
}

TEST(TutorialTest, p1732)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue);
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	a.start();
	
    print(PRI1, "p1732(): completionflow1732();\n");
    completionflow1732();

    print(PRI1, "p1732(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1732(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1734()
{

}

TEST(TutorialTest, p1734)
{
    set_print_level(0x00);

    std::mutex mtx;
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &mtx);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &mtx);
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	a.start();
	
    print(PRI1, "p1734(): completionflow1734();\n");
    completionflow1734();

    print(PRI1, "p1734(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1734(): v = %d\n", v);

    // Added timer to avoid the following sporadic error:
    // D:\a\_work\1\s\src\vctools\crt\github\stl\src\mutex.cpp(49): mutex destroyed while busy
    print(PRI1, "p1734(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1735()
{
    print(PRI1, "completionflow1735(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow1735():runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);
}

TEST(TutorialTest, p1735)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class01 object02(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	a.start();
	
    print(PRI1, "p1735(): completionflow1735();\n");
    completionflow1735();

    print(PRI1, "p1735(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1735(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// ---------------------------------

void completionflow1736()
{

}

TEST(TutorialTest, p1736)
{
    set_print_level(0x00);

    Class01 object01(UseMode::USE_IMMEDIATE_COMPLETION);
    Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
    Class1730 obj{ object01, object02 };
    async_ltask<int> a = obj.coroutine1();
	a.start();
	
    print(PRI1, "p1736(): completionflow1736();\n");
    completionflow1736();

    print(PRI1, "p1736(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1736(): v = %d\n", v);

    ASSERT_EQ(v, 49);
}

// -------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
