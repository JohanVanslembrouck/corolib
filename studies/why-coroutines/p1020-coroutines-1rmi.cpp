/**
 * @file p1020-coroutines-1rmi.cpp
 * @brief Coroutine variant of p1000-sync-1rmi.cpp.
 * This file defines 3 classes.
 * 
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "p1000co.h"

/**
 * @brief
 * Class01a defines coroutine1 and coroutine1a.
 * Both coroutines have in-parameters only.
 * 
 * Both coroutines call RemoteObject1Co::start_op1 that takes in-parameters only
 * and that returns an op1_ret_t struct containing the out-parameters and return value
 * of the original RemoteObject1 function.
 * 
 * coroutine1a uses an intermediate async_operation<op1_ret_t> op1 object.
 * This shows that RemoteObject1Co::start_op1 is a "normal" function, not a coroutine.
 * 
 */
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

/**
 * @brief
 * Class01 also defines coroutine1 and coroutine1a.
 * Both coroutines have in-parameters only.
 * 
 * Both coroutines call RemoteObject1Co::op1 that has the same signature
 * as the original RemoteObject1 function (see p1000-sync-1rmi.cpp).
 *
 * coroutine1a uses an intermediate async_task<int> op1 object.
 * This shows that RemoteObject1Co::op1 is a coroutine, not a "normal" function.
 * 
 */
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

/**
 * @brief
 * Class01b again defines coroutine1 and coroutine1a.
 * Both coroutines have two in-parameters and two out-parameters,
 * making the out-parameters available to the caller of both coroutines.
 * 
 * As in Class01, both coroutines call RemoteObject1Co::op1 that has the same signature
 * as the original RemoteObject1 function.
 *
 * coroutine1a uses an intermediate async_task<int> op1 object.
 * This shows that RemoteObject1Co::op1 is a coroutine, not a "normal" function.
 * 
 */
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
    printf("main(): begin\n");
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
    int ret2 = t2.get_result();
    int ret3 = t3.get_result();
    int ret4 = t4.get_result();
    int ret5 = t5.get_result();
    int ret6 = t6.get_result();

    printf("\n");
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    printf("main(): ret5 = %d\n", ret5);
    printf("main(): ret6 = %d\n", ret6);
    printf("main(): end\n");
    return 0;
}
