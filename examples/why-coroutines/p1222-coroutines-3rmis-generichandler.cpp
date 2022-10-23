/**
 * @file p1222-coroutines-3rmis-generichandler.cpp
 * @brief Variant of p1220 using a generic version of the completion handler.
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

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
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1()\n");
        op1_ret_t res1 = co_await remoteObj1co.start_op1(gin11, gin12);
        // 1 Do stuff
        if (res1.ret == gval1) {
            op2_ret_t res1 = co_await remoteObj2co.start_op2(gin21, gin22);
            (void)res1;
            // 2 Do stuff
        }
        else {
            op1_ret_t res3 = co_await remoteObj3co.start_op3(gin31);
            (void)res3;
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        printf("Class01a::coroutine1a()\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(gin11, gin12);
        // 1a Do some stuff that doesn't need the result of the RMI
        op1_ret_t res1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (res1.ret == gval1) {
            async_operation<op2_ret_t> op2 = remoteObj2co.start_op2(gin21, gin22);
            // 2a Do some stuff that doesn't need the result of the RMI
            op2_ret_t res1 = co_await op2;
            (void)res1;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3co.start_op3(gin31);
            // 3a Do some stuff that doesn't need the result of the RMI
            op1_ret_t res3 = co_await op3;
            (void)res3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

struct Class01
{
    async_task<void> coroutine1()
    {
        int ret1 = co_await remoteObj1co.op1(gin11, gin12, gout11, gout12);
        // 1 Do stuff
        if (ret1 == gval1) {
            int ret2 = co_await remoteObj2co.op2(gin21, gin22, gout21);
            (void)ret2;
            // 2 Do stuff
        }
        else {
            int ret3 = co_await remoteObj3co.op3(gin31, gout31, gout32);
            (void)ret3;
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        async_task<int> op1 = remoteObj1co.op1(gin11, gin12, gout11, gout12);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            async_task<int> op2 = remoteObj2co.op2(gin21, gin22, gout21);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            (void)ret2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_task<int> op3 = remoteObj3co.op3(gin31, gout31, gout32);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            (void)ret3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

Class01 class01;

EventQueue eventQueue;

int main() {
    printf("main();\n");
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
