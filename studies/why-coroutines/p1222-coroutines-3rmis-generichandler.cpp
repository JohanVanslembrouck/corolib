/**
 * @file p1222-coroutines-3rmis-generichandler.cpp
 * @brief Variant of p1220 using a generic version of the completion handler.
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"
#include "p1200.h"
#include "p1200cog.h"

using namespace corolib;

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

RemoteObject1Co remoteObj1co{ remoteObj1 };
RemoteObject1Co remoteObj2co{ remoteObj2 };
RemoteObject1Co remoteObj3co{ remoteObj3 };

class Class01a
{
public:
    async_task<void> coroutine1(int in1, int in2, int testval)
    {
        printf("Class01a::coroutine1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        op1_ret_t res1 = co_await remoteObj1co.start_op1(in1, in2);
        printf("Class01a::coroutine1: 1: out1 = %d, out2 = %d, ret1 = %d\n", res1.out1, res1.out2, res1.ret);
        // 1 Do stuff
        if (res1.ret == testval) {
            op2_ret_t res2 = co_await remoteObj2co.start_op2(in1, in2);
            printf("Class01a::coroutine1: 2: out3 = %d, ret2 = %d\n", res2.out1, res2.ret);
            // 2 Do stuff
        }
        else {
            op1_ret_t res3 = co_await remoteObj3co.start_op3(in1);
            printf("Class01a::coroutine1: 3: out4 = %d, out5 = %d, ret3 = %d\n", res3.out1, res3.out2, res3.ret);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a(int in1, int in2, int testval)
    {
        printf("Class01a::coroutine1a(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(in1, in2);
        // 1a Do some stuff that doesn't need the result of the RMI
        op1_ret_t res1 = co_await op1;
        printf("Class01a::coroutine1a: 1: out1 = %d, out2 = %d, ret1 = %d\n", res1.out1, res1.out2, res1.ret);
        // 1b Do stuff that needs the result of the RMI
        if (res1.ret == testval) {
            async_operation<op2_ret_t> op2 = remoteObj2co.start_op2(in1, in2);
            // 2a Do some stuff that doesn't need the result of the RMI
            op2_ret_t res2 = co_await op2;
            printf("Class01a::coroutine1a: 2: out3 = %d, ret2 = %d\n", res2.out1, res2.ret);
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3co.start_op3(in1);
            // 3a Do some stuff that doesn't need the result of the RMI
            op1_ret_t res3 = co_await op3;
            printf("Class01a::coroutine1a: 3: out4 = %d, out5 = %d, ret3 = %d\n", res3.out1, res3.out2, res3.ret);
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

struct Class01
{
    async_task<void> coroutine1(int in1, int in2, int testval)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01::coroutine1: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1 Do stuff
        if (ret1 == testval) {
            int out3 = -1;
            int ret2 = co_await remoteObj2co.op2(in1, in2, out3);
            printf("Class01::coroutine1: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = co_await remoteObj3co.op3(in1, out4, out5);
            printf("Class01::coroutine1: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a(int in1, int in2, int testval)
    {
        printf("Class01::coroutine1a(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        async_task<int> op1 = remoteObj1co.op1(in1, in2, out1, out2);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        printf("Class01::coroutine1a: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == testval) {
            int out3 = -1;
            async_task<int> op2 = remoteObj2co.op2(in1, in2, out3);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            printf("Class01::coroutine1a: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            int out4 = -1, out5 = -1;
            async_task<int> op3 = remoteObj3co.op3(in1, out4, out5);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            printf("Class01::coroutine1a: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

Class01 class01;
Class01a class01a;

EventQueue eventQueue;

int main() {
    printf("main();\n");
#if 1
    async_task<void> t1 = class01.coroutine1(11, 12, 10);
    eventQueue.run();
    t1.wait();
    printf("\n");

    async_task<void> t2 = class01.coroutine1(11, 12, 23);
    eventQueue.run();
    t2.wait();
    printf("\n");

    async_task<void> t3 = class01a.coroutine1(11, 12, 10);
    eventQueue.run();
    t3.wait();
    printf("\n");

    async_task<void> t4 = class01a.coroutine1(11, 12, 23);
    eventQueue.run();
    t4.wait();
    printf("\n");
#else
    async_task<void> t1 = class01.coroutine1(11, 12, 10);
    async_task<void> t2 = class01.coroutine1(11, 12, 23);
    eventQueue.run();
    t1.wait();
    t2.wait();

    async_task<void> t3 = class01a.coroutine1(11, 12, 10);
    async_task<void> t4 = class01a.coroutine1(11, 12, 23);
    eventQueue.run();
    t3.wait();
    t4.wait();
#endif
    return 0;
}
