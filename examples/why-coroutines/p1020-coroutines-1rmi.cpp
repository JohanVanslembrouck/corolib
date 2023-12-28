/**
 * @file p1020-coroutines-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "variables.h"

#include "p1000co.h"

RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{remoteObj1};

class Class01a
{
public:
    async_task<int> coroutine1()
    {
        printf("Class01a::coroutine1() - part 1\n");
        op1_ret_t ret = co_await remoteObj1co.start_op1(gin11, gin12);
        printf("Class01a::coroutine1(): ret.out1 = %d, ret.out2 = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        printf("Class01a::coroutine1() - part 2\n");
        co_return ret.ret;
    }
    
    async_task<int> coroutine1a()
    {
        printf("Class01a::coroutine1a() - part 1\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(gin11, gin12);
        op1_ret_t ret = co_await op1;
        printf("Class01a::coroutine1a(): ret.out1 = %d, ret.outé = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        printf("Class01a::coroutine1a() - part 2\n");
        co_return ret.ret;
    }
};

Class01a class01a;

class Class01
{
public:
    async_task<int> coroutine1()
    {
        printf("Class01::coroutine1() - part 1\n");
        int ret1 = co_await remoteObj1co.op1(gin11, gin12, gout11, gout12);
        printf("Class01::coroutine1(): gout11 = %d, gout12 = %d, ret1 = %d\n", gout11, gout12, ret1);
        printf("Class01::coroutine1() - part 2\n");
        co_return ret1;
    }
    
    async_task<int> coroutine1a()
    {
        printf("Class01::coroutine1a() - part 1\n");
        async_task<int> op1 = remoteObj1co.op1(gin11, gin12, gout11, gout12);
        int ret1 = co_await op1;
        printf("Class01::coroutine1a(): gout11 = %d, gout12 = %d, ret1 = %d\n", gout11, gout12, ret1);
        printf("Class01::coroutine1a() - part 2\n");
        co_return ret1;
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    async_task<int> t1 = class01a.coroutine1();
    async_task<int> t2 = class01a.coroutine1a();
    async_task<int> t3 = class01.coroutine1();
    async_task<int> t4 = class01.coroutine1a();
    eventQueue.run();
    return 0;
}
