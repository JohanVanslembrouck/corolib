/**
 * @file p11XX_tests.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "p1100-auto_reset_event-1.h"
#include "p1101-auto_reset_event-1-when_all.h"
#include "p1102-auto_reset_event-1-when_any.h"
#include "p1103-auto_reset_event-2.h"
#include "p1104-auto_reset_event-2-when_all.h"
#include "p1105-auto_reset_event-2-when_any.h"
#include "p1106-auto_reset_event-3.h"
#include "p1107-auto_reset_event-3-when_all.h"
#include "p1108-auto_reset_event-3-when_any.h"

#include "p1110-auto_reset_event-thread-1.h"
#include "p1111-auto_reset_event-thread-1-when_all.h"
#include "p1112-auto_reset_event-thread-1-when_any.h"
#include "p1113-auto_reset_event-thread-2.h"
#include "p1114-auto_reset_event-thread-2-when_all.h"
#include "p1115-auto_reset_event-thread-2-when_any.h"
#include "p1116-auto_reset_event-thread-3.h"
#include "p1117-auto_reset_event-thread-3-when_all.h"
#include "p1118-auto_reset_event-thread-3-when_any.h"

#include "gtest/gtest.h"

using namespace corolib;

auto_reset_event are1;
auto_reset_event are2;
auto_reset_event are3;

TEST(TutorialTest, p1100)
{
    set_print_level(0x00);

    Class1100 obj;
    print(PRI1, "p1100(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
	
    print(PRI1, "p1100(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1100(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "p1100(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1100(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1101)
{
    set_print_level(0x00);

    Class1101 obj;
    print(PRI1, "p1101(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
	
    print(PRI1, "p1101(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1101(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "p1101(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1101(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1102)
{
    set_print_level(0x00);

    Class1102 obj;
    print(PRI1, "p1102(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1102(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1102(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1102(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1102(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1103)
{
    set_print_level(0x00);

    Class1103 obj;
    print(PRI1, "p1103(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1103(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1103(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1103(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1103(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1103(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1103(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1104)
{
    set_print_level(0x00);

    Class1104 obj;
    print(PRI1, "p1104(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1104(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1104(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1104(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1104(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1104(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1104(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1105)
{
    set_print_level(0x00);

    Class1105 obj;
    print(PRI1, "p1105(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1105(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1105(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1105(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1105(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1105(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1105(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1106)
{
    set_print_level(0x00);

    Class1106 obj;
    print(PRI1, "p1106(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1106(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1106(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1106(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1106(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1106(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1106(): are3.resume();\n");
    are3.resume();

    print(PRI1, "p1106(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1106(): v = %d\n", v);
    
    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1107)
{
    set_print_level(0x00);

    Class1107 obj;
    print(PRI1, "p1107(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1107(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1107(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1107(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1107(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1107(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1107(): are3.resume();\n");
    are3.resume();

    print(PRI1, "p1107(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1107(): v = %d\n", v);

    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1108)
{
    set_print_level(0x00);

    Class1108 obj;
    print(PRI1, "p1108(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "p1108(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1108(): are1.resume();\n");
    are1.resume();

    print(PRI1, "p1108(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1108(): are2.resume();\n");
    are2.resume();

    print(PRI1, "p1108(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "p1108(): are3.resume();\n");
    are3.resume();

    print(PRI1, "p1108(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1108(): v = %d\n", v);

    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1110)
{
    set_print_level(0x00);

    Class1110 obj;
    print(PRI1, "p1110(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1110(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1110(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1110a)
{
    set_print_level(0x00);

    ThreadAwaker awaker;
    Class1110 obj{ &awaker, 0 };
    print(PRI1, "p1110a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1110a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1110a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1110a(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1111)
{
    set_print_level(0x00);

    Class1111 obj;
    print(PRI1, "p1111(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1111(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1111(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1111a)
{
    set_print_level(0x00);

    ThreadAwaker awaker;
    Class1111 obj{ &awaker, 0 };
    print(PRI1, "p1111a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1111a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1111a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1111a(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1112)
{
    set_print_level(0x00);

    Class1112 obj;
    print(PRI1, "p1112(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1112(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1112(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1112a)
{
    set_print_level(0x00);

    ThreadAwaker awaker;
    Class1112 obj{ &awaker, 0 };
    print(PRI1, "p1112a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1112a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1112a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1112a(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1113)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1113 obj{ &mutx };
    print(PRI1, "p1113(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1113(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1113(): v = %d\n", v);

    print(PRI1, "p1113(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1113a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1113 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1113a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1113a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1113a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1113a(): v = %d\n", v);

    print(PRI1, "p1113a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1114)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1114 obj{ &mutx };
    print(PRI1, "p1114(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1114(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1114(): v = %d\n", v);

    print(PRI1, "p1114(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1114a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1114 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1114a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1114a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1114a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1114a(): v = %d\n", v);

    print(PRI1, "p1114a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1115)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1115 obj{ &mutx };
    print(PRI1, "p1115(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1115(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1115(): v = %d\n", v);

    print(PRI1, "p1115(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1115a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1115 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1115a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1115a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1115a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1115a(): v = %d\n", v);

    print(PRI1, "p1115a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1116)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1116 obj{ &mutx };
    print(PRI1, "p1116(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1116(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1116(): v = %d\n", v);

    print(PRI1, "p1116(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1116a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1116 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1116a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1116a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1116a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1116a(): v = %d\n", v);

    print(PRI1, "p1116a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1117)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1117 obj{ &mutx };
    print(PRI1, "p1117(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1117(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1117(): v = %d\n", v);

    print(PRI1, "p1117(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1117a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1117 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1117a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1117a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1117a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1117a(): v = %d\n", v);

    print(PRI1, "p1117a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1118)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1118 obj{ &mutx };
    print(PRI1, "p1118(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1118(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1118(): v = %d\n", v);

    print(PRI1, "p1118(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1118a)
{
    set_print_level(0x00);

    std::mutex mutx;
    ThreadAwaker awaker;
    Class1118 obj{ &mutx, &awaker, 0 };
    print(PRI1, "p1118a(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "p1118a(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "p1118a(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "p1118a(): v = %d\n", v);

    print(PRI1, "p1118a(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

// -------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
