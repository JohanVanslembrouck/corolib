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
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
	
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1101)
{
    set_print_level(0x00);

    Class1101 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
	
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1102)
{
    set_print_level(0x00);

    Class1102 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1103)
{
    set_print_level(0x00);

    Class1103 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1104)
{
    set_print_level(0x00);

    Class1104 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1105)
{
    set_print_level(0x00);

    Class1105 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1106)
{
    set_print_level(0x00);

    Class1106 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are3.resume();\n");
    are3.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    
    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1107)
{
    set_print_level(0x00);

    Class1107 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are3.resume();\n");
    are3.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1108)
{
    set_print_level(0x00);

    Class1108 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are2.resume();\n");
    are2.resume();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are3.resume();\n");
    are3.resume();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 8);
}

TEST(TutorialTest, p1110)
{
    set_print_level(0x00);

    Class1110 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1111)
{
    set_print_level(0x00);

    Class1111 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1112)
{
    set_print_level(0x00);

    Class1112 obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    ASSERT_EQ(v, 4);
}

TEST(TutorialTest, p1113)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1113 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1114)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1114 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1115)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1115 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 5);
}

TEST(TutorialTest, p1116)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1116 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1117)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1117 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

TEST(TutorialTest, p1118)
{
    set_print_level(0x00);

    std::mutex mutx;
    Class1118 obj{ &mutx };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(v, 6);
}

// -------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
