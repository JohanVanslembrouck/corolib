/**
 * @file p1020-coroutines-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "p1000co.h"

class Class01a
{
public:
    async_task<int> coroutine1(int in1, int in2)
    {
        printf("Class01a::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        op1_ret_t ret = co_await remoteObj1co.start_op1(in1, in2);
        printf("Class01a::coroutine1(): ret.out1 = %d, ret.out2 = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        co_return in1 + in2 + ret.out1 + ret.out2 + ret.ret;
    }
    
    async_task<int> coroutine1a(int in1, int in2)
    {
        printf("Class01a::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(in1, in2);
        op1_ret_t ret = co_await op1;
        printf("Class01a::coroutine1a(): ret.out1 = %d, ret.out2 = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        co_return in1 + in2 + ret.out1 + ret.out2 + ret.ret;
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1Co remoteObj1co{ remoteObj1 };
};

class Class01
{
public:
    async_task<int> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + in2 + out1 + out2 + ret1;
    }
    
    async_task<int> coroutine1a(int in1, int in2)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        async_task<int> op1 = remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01::coroutine1a(): out1 = %d, out2 = %d\n", out1, out2);
        int ret1 = co_await op1;
        printf("Class01::coroutine1a(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + in2 + out1 + out2 + ret1;
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1Co remoteObj1co{ remoteObj1 };
};

class Class01b
{
public:
    async_task<int> coroutine1(int in1, int in2, int& out1, int& out2)
    {
        printf("Class01b::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01b::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + in2 + out1 + out2 + ret1;
    }

    async_task<int> coroutine1a(int in1, int in2, int& out1, int& out2)
    {
        printf("Class01b::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        async_task<int> op1 = remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01b::coroutine1a(): out1 = %d, out2 = %d\n", out1, out2);
        int ret1 = co_await op1;
        printf("Class01b::coroutine1a(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + in2 + out1 + out2 + ret1;
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1Co remoteObj1co{ remoteObj1 };
};

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    Class01a class01a;
    Class01 class01;
    Class01b class01b;
    async_task<int> t1 = class01a.coroutine1(11, 12);
    async_task<int> t2 = class01a.coroutine1a(21, 22);
    async_task<int> t3 = class01.coroutine1(31, 32);
    async_task<int> t4 = class01.coroutine1a(41, 42);
    int out11 = -1, out12 = -1, out21 = -1, out22 = -1;
    async_task<int> t5 = class01b.coroutine1(31, 32, out11, out12);
    async_task<int> t6 = class01b.coroutine1a(41, 42, out21, out22);
    eventQueue.run();
    printf("\n");

    int ret1 = t1.get_result();
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = t2.get_result();
    printf("main(): ret2 = %d\n", ret2);
    int ret3 = t3.get_result();
    printf("main(): ret3 = %d\n", ret3);
    int ret4 = t4.get_result();
    printf("main(): ret4 = %d\n", ret4);
    int ret5 = t5.get_result();
    printf("main(): ret5 = %d\n", ret5);
    int ret6 = t6.get_result();
    printf("main(): ret6 = %d\n", ret6);
    return 0;
}
