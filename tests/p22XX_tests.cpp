/**
 * @file p22XX_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <thread>

#include <corolib/tracker.h>
#include <corolib/eventqueue.h>
#include <corolib/print.h>

using namespace corolib;

#include "gtest/gtest.h"

const int NR_ITERATIONS = 10000;

using EventQueueInt = QueueThreadSafe<int, ARRAYSIZE>;

EventQueueInt queue;

int nrErrors = 0;
bool resultOK = true;

// ---------------------------------------------------------

void task1a(int div)
{
    print(PRI1, "task1: enter\n");
    for (int i = 0; i < NR_ITERATIONS; ++i)
    {
        if (div && i % div == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        queue.push(i);
    }
    print(PRI1, "task1: exit\n");
}

void task2a(int div)
{
    int resOld = -1;
    print(PRI1, "task2: enter\n");
    for (int i = 0; i < NR_ITERATIONS; ++i)
    {
        if (div && i % 20 == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        int res = queue.pop();
        if (res != resOld + 1) {
            print(PRI1, "task2: received %d, expected %d\n", res, resOld + 1);
            nrErrors++;
        }
        resOld = res;
    }
    print(PRI1, "task2: exit\n");
}

void task3(int div)
{
    print(PRI1, "task3: enter\n");
    int res = -1;
    for (int i = 0; i < 3 * NR_ITERATIONS; ++i)
    {
        if (div && i % div == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        res = queue.pop();
    }
    if (res != NR_ITERATIONS - 1) {
        print(PRI1, "task3: received %d, expected %d\n", res, NR_ITERATIONS - 1);
        resultOK = false;
    }
    print(PRI1, "task3: exit\n");
}

// ---------------------------------------------------------

void task1b(int div)
{
    print(PRI1, "task1: enter\n");
    for (int i = 0; i < NR_ITERATIONS; ++i)
    {
        if (div && i % div == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        queue.push(i);
    }
    queue.pushFinal(NR_ITERATIONS);
    print(PRI1, "task1: exit\n");
}

void task2b(int div)
{
    int resOld = -1;
    int i = 0;
    queue.reset();
    print(PRI1, "task2: enter\n");
    do
    {
        if (div && i++ % div == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        int res = queue.pop();
        if (res != resOld + 1) {
            print(PRI1, "task2: received %d, expected %d\n", res, resOld + 1);
            nrErrors++;
        }
        resOld = res;
    } while (!queue.stopped());
    int length = queue.length();
    for (int i = 0; i < length; i++)
    {
        int res = queue.pop();
        if (res != resOld + 1) {
            print(PRI1, "task2: received %d, expected %d\n", res, resOld + 1);
            nrErrors++;
        }
        resOld = res;
    }
    print(PRI1, "task2: exit\n");
}

// ---------------------------------------------------------

void startTasksA(int a, int b)
{
    print(PRI1, "main(): std::jthread task1thr{ task1a, %d };\n", a);
    std::jthread task1thr{ task1a, a };
    print(PRI1, "main(): std::jthread task2thr{ task2a, %d };\n", b);
    std::jthread task2thr{ task2a, b };
}

void startTasksB(int a, int b)
{
    print(PRI1, "main(): std::jthread task1thr{ task1b, %d };\n", a);
    std::jthread task1thr{ task1b, a };
    print(PRI1, "main(): std::jthread task2thr{ task2b, %d };\n", b);
    std::jthread task2thr{ task2b, b };
}

void startTasksC(int a, int b)
{
    print(PRI1, "main(): std::jthread task1athr{ task1a, a };\n");
    std::jthread task1athr{ task1a, a };
    print(PRI1, "main(): std::jthread task1bthr{ task1a, a };\n");
    std::jthread task1bthr{ task1a, a };
    print(PRI1, "main(): std::jthread task1cthr{ task1a, a };\n");
    std::jthread task1cthr{ task1a, a };

    print(PRI1, "main(): std::jthread task2thr{ task2 };\n");
    std::jthread task2thr{ task3, b };
}

// ---------------------------------------------------------

TEST(TutorialTest, p2200)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksA(0, 0);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2201)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksB(0, 0);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2202)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksA(20, 0);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2203)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksB(20, 0);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2204)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksA(0, 20);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2205)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksB(0, 20);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2206)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksA(30, 20);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2207)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksB(30, 20);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2208)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksA(20, 30);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

TEST(TutorialTest, p2209)
{
    set_print_level(0x00);
    nrErrors = 0;

    startTasksB(20, 30);
    print(PRI1, "nrErrors = %d\n", nrErrors);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(nrErrors, 0);
}

// ---------------------------------------------------------

TEST(TutorialTest, p2210)
{
    set_print_level(0x00);
    resultOK = true;

    startTasksC(0, 0);
    print(PRI1, "resultOK = %d\n", resultOK);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(resultOK, true);
}

TEST(TutorialTest, p2212)
{
    set_print_level(0x00);
    resultOK = true;

    startTasksC(20, 0);
    print(PRI1, "resultOK = %d\n", resultOK);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(resultOK, true);
}

TEST(TutorialTest, p2214)
{
    set_print_level(0x00);
    resultOK = true;

    startTasksC(0, 20);
    print(PRI1, "resultOK = %d\n", resultOK);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(resultOK, true);
}

TEST(TutorialTest, p2216)
{
    set_print_level(0x00);
    resultOK = true;

    startTasksC(20, 30);
    print(PRI1, "resultOK = %d\n", resultOK);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(resultOK, true);
}

TEST(TutorialTest, p2218)
{
    set_print_level(0x00);
    resultOK = true;

    startTasksC(30, 20);
    print(PRI1, "resultOK = %d\n", resultOK);
    print(PRI1, "main(): return 0;\n");

    ASSERT_EQ(resultOK, true);
}

// ---------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
