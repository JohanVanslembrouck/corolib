/** 
 * @file p1300-future.cpp
 * @brief
 *
 * Instead of using a user-defined coroutine type (as in most other examples),
 * this example uses the future type that Microsoft has extended 
 * with coroutine functionality and comes with the installation of Microsoft Studio 2019.
 *
 * This implementation spawns a lot of additional threads, probably because this was the only way
 * to avoid modifications to the existing implementation of future/promise.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <corolib/print.h>

using namespace corolib;

#include <future>

using namespace std;

future<int> coroutine5(int a, int b)
{
#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    print(PRI1, "coroutine5(%d, %d) 1: auto f6 = std::async([=]() {...});\n", a, b);
    future<int> f6 = std::async([=]() {
        print(PRI1, "std::async(...) 1: int c = %d + %d;\n", a, b);
        int c = a + b;
        print(PRI1, "std::async(...) 2: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print(PRI1, "std::async(...) 3: return c = %d;\n", c);
        return c;
        });

    print(PRI1, "coroutine5(%d, %d) 2: int c = co_await f6;\n", a, b);
    int c = co_await f6;
#else
    print(PRI1, "coroutine5(%d, %d) 1: int c = %d + %d;\n", a, b);
    int c = a + b;
#endif

    print(PRI1, "coroutine5(%d, %d) 3: int d = a + c + 5;\n", a, b);
    int d = a + c + 5;
    print(PRI1, "coroutine5(%d, %d) 4: co_return d = %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine4(int a, int b)
{
    print(PRI1, "coroutine4(%d, %d) 1: future<int> f5 = coroutine5(a, b);\n", a, b);
    future<int> f5 = coroutine5(a, b);
    print(PRI1, "coroutine4(%d, %d) 2: int c = co_await f5;\n", a, b);
    int c = co_await f5;
    print(PRI1, "coroutine4(%d, %d) 3: int d = a + c + 4;\n", a, b);
    int d = a + c + 4;
    print(PRI1, "coroutine4(%d, %d) 4: co_return d = %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine3(int a, int b)
{
    print(PRI1, "coroutine3(%d, %d) 1: future<int> f4 = coroutine4(a, b);\n", a, b);
    future<int> f4 = coroutine4(a, b);
    print(PRI1, "coroutine3(%d, %d) 2: int c = co_await f4;\n", a, b);
    int c = co_await f4;
    print(PRI1, "coroutine3(%d, %d) 3: int d = a + c + 3;\n", a, b);
    int d = a + c + 3;
    print(PRI1, "coroutine3(%d, %d) 4: co_return d = %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine2(int a, int b)
{
    int d = 0;
    for (int i = 0; i < 5; i++)
    {
        print(PRI1, "coroutine2(%d, %d) 1: future<int> f3 = coroutine3(a, b);\n", a, b);
        future<int> f3 = coroutine3(a, b);
        print(PRI1, "coroutine2(%d, %d) 2: int c = co_await f3;\n", a, b);
        int c = co_await f3;
        print(PRI1, "coroutine2(%d, %d) 3: int d = a + c + 2;\n", a, b);
        d = a + c + 2;
        print(PRI1, "coroutine2(%d, %d) 4: co_return d = %d;\n", a, b, d);
    }
    co_return d;
}

future<int> coroutine1(int a, int b)
{
    print(PRI1, "coroutine1(%d, %d) 1: future<int> f2 = coroutine2(a, b);\n", a, b);
    future<int> f2 = coroutine2(a, b);
    print(PRI1, "coroutine1(%d, %d) 2: int c = co_await fc;\n", a, b);
    int c = co_await f2;
    print(PRI1, "coroutine1(%d, %d) 3: int d = a + c + 1;\n", a, b);
    int d = a + c + 1;
    print(PRI1, "coroutine1(%d, %d) 4: co_return d = %d;\n", a, b, d);
    co_return d;
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main() 1: future<int> f1 = coroutine1(%d, %d);\n", 1, 3);
    future<int> f1 = coroutine1(1, 3);
    print(PRI1, "main() 2: f1.wait();\n");
    f1.wait();
    print(PRI1, "main() 3: return 0;\n");
    return 0;
}
