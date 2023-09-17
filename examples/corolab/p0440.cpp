/** 
 *  Filename: p0440.cpp
 *  Description: 
 *        Illustrates the use of co_await.
 *
 *        Instead of using a user-defined coroutine type (as in most other examples),
 *        this example uses the future type that Microsoft has extended 
 *        with coroutine functionality and comes with the installation of 
 *        Microsoft Studio 2019.
 *
 *        This implementation spawns a lot of additional threads
 *        (see trace), probably because this was the only way
 *        to avoid modifications to the existing implementation
 *        of future/promise.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string>

#include <future>

using namespace std;

#include "print.h"

//--------------------------------------------------------------

future<int> coroutine5(int a, int b)
{
#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    print("coroutine5(%d, %d) 1: auto f6 = std::async([=]() {...});\n", a, b);
    future<int> f6 = std::async([=]() {
        print("std::async(...) 1: int c = %d + %d;\n", a, b);
        int c = a + b;
        print("std::async(...) 2: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("std::async(...) 3: return c = %d;\n", c);
        return c;
        });

    print("coroutine5(%d, %d) 2: int c = co_await f6; : &f6 = %p\n", a, b, &f6);
    int c = co_await f6;
#else
    print("coroutine5(%d, %d) 1: int c = %d + %d;\n", a, b);
    int c = a + b;
#endif

    print("coroutine5(%d, %d) 3: int d = a + c + 5;\n", a, b);
    int d = a + c + 5;
    print("coroutine5(%d, %d) 4: co_return %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine4(int a, int b)
{
    print("coroutine4(%d, %d) 1: future<int> f5 = coroutine5(a, b);\n", a, b);
    future<int> f5 = coroutine5(a, b);
    print("coroutine4(%d, %d) 2: int c = co_await f5; : &f5 = %p\n", a, b, &f5);
    int c = co_await f5;
    print("coroutine4(%d, %d) 3: int d = a + c + 4;\n", a, b);
    int d = a + c + 4;
    print("coroutine4(%d, %d) 4: co_return %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine3(int a, int b)
{
    print("coroutine3(%d, %d) 1: future<int> f4 = coroutine4(a, b);\n", a, b);
    future<int> f4 = coroutine4(a, b);
    print("coroutine3(%d, %d) 2: int c = co_await f4; : &f4 = %p\n", a, b, &f4);
    int c = co_await f4;
    print("coroutine3(%d, %d) 3: int d = a + c + 3;\n", a, b);
    int d = a + c + 3;
    print("coroutine3(%d, %d) 4: co_return %d;\n", a, b, d);
    co_return d;
}

future<int> coroutine2(int a, int b)
{
    int d = 0;
    for (int i = 0; i < 5; i++)
    {
        print("coroutine2(%d, %d) 1: future<int> f3 = coroutine3(a, b);\n", a, b);
        future<int> f3 = coroutine3(a, b);
        print("coroutine2(%d, %d) 2: int c = co_await f3; : &f3 = %p\n", a, b, &f3);
        int c = co_await f3;
        print("coroutine2(%d, %d) 3: int d = a + c + 2;\n", a, b);
        d = a + c + 2;
        print("coroutine2(%d, %d) 4: co_return %d;\n", a, b, d);
    }
    co_return d;
}

future<int> coroutine1(int a, int b)
{
    print("coroutine1(%d, %d) 1: future<int> f2 = coroutine2(a, b);\n", a, b);
    future<int> f2 = coroutine2(a, b);
    print("coroutine1(%d, %d) 2: int c = co_await fc; : &f2 = %p\n", a, b, &f2);
    int c = co_await f2;
    print("coroutine1(%d, %d) 3: int d = a + c + 1;\n", a, b);
    int d = a + c + 1;
    print("coroutine1(%d, %d) 4: co_return %d;\n", a, b, d);
    co_return d;
}

int main()
{
    print("main() 1: future<int> f1 = coroutine1(%d, %d);\n", 1, 3);
    future<int> f1 = coroutine1(1, 3);
    print("main() 2: f1.wait(); : &f1 = %p\n", &f1);
    f1.wait();
    print("main() 3: return 0;\n");
    return 0;
}

