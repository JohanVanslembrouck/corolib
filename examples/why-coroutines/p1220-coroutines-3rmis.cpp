/**
 * @file p1220-coroutines-3rmis.cpp
 * @brief
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
#include "p1200co.h"

using namespace corolib;

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

RemoteObject1Co remoteObj1co{remoteObj1};
RemoteObject1Co remoteObj2co{remoteObj2};
RemoteObject1Co remoteObj3co{remoteObj3};

int gval1 = 0;

class Class01a
{
public:
    async_task<void> coroutine1(int in1, int in2)
    {
        printf("Class01a::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        op1_ret_t res1 = co_await remoteObj1co.start_op1(in1, in2);
        // 1 Do stuff
        if (res1.ret == gval1) {
            op2_ret_t res1 = co_await remoteObj2co.start_op2(in1, in2);
            (void)res1;
            // 2 Do stuff
        }
        else {
            op1_ret_t res3 = co_await remoteObj3co.start_op3(in1);
            (void)res3;
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a(int in1, int in2)
    {
        printf("Class01a::coroutine1a(in1 = %d, in2 = %d)\n", in1, in2);
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(in1, in2);
        // 1a Do some stuff that doesn't need the result of the RMI
        op1_ret_t res1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (res1.ret == gval1) {
            async_operation<op2_ret_t> op2 = remoteObj2co.start_op2(in1, in2);
            // 2a Do some stuff that doesn't need the result of the RMI
            op2_ret_t res1 = co_await op2;
            (void)res1;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3co.start_op3(in1);
            // 3a Do some stuff that doesn't need the result of the RMI
            op1_ret_t res3 = co_await op3;
            (void)res3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

struct Class01
{
    async_task<void> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        // 1 Do stuff
        if (ret1 == gval1) {
            int out3 = -1;
            int ret2 = co_await remoteObj2co.op2(in1, in2, out3);
            (void)ret2;
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = co_await remoteObj3co.op3(in1, out4, out5);
            (void)ret3;
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a(int in1, int in2)
    {
        printf("Class01::coroutine1a(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        async_task<int> op1 = remoteObj1co.op1(in1, in2, out1, out2);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            int out3 = -1;
            async_task<int> op2 = remoteObj2co.op2(in1, in2, out3);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            (void)ret2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            int out4 = -1, out5 = -1;
            async_task<int> op3 = remoteObj3co.op3(in1, out4, out5);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            (void)ret3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    async_task<void> t1 = class01.coroutine1(11, 12);
    async_task<void> t2 = class01.coroutine1(21, 22);
    eventQueue.run();
    t1.wait();
    t2.wait();
    return 0;
}
