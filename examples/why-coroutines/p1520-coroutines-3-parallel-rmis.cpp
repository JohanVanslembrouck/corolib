/**
 * @file p1520-coroutines-3-parallel-rmis.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1200co.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

RemoteObject1Co remoteObj1co{remoteObj1};
RemoteObject1Co remoteObj2co{remoteObj2};
RemoteObject1Co remoteObj3co{remoteObj3};

class Class01a
{
public: 
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1()\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(gin11, gin12);
        async_operation<op1_ret_t> op2 = remoteObj2co.start_op1(gin11, gin12);
        async_operation<op1_ret_t> op3 = remoteObj3co.start_op1(gin11, gin12);
#if 0
        // g++ 11 does not like this one-liner
        co_await when_all<async_operation<op1_ret_t>>({ &op1, &op2, &op3 });
#else
        when_all<async_operation<op1_ret_t>> wa({ &op1, &op2, &op3 });
        co_await wa;
#endif
        printf("Class01a::coroutine1(); result = %d\n", op1.get_result().ret +  op2.get_result().ret + op3.get_result().ret);
    } // g++ 11 reports at this line: error: array used as initializer
};

Class01a class01a;

class Class01
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01::coroutine1()\n");
        async_task<int> op1 = remoteObj1co.op1(gin11, gin12, gout11, gout12);
        async_task<int> op2 = remoteObj2co.op1(gin11, gin12, gout11, gout12);
        async_task<int> op3 = remoteObj3co.op1(gin11, gin12, gout11, gout12);
#if 0
        // g++ 11 does not like this one-liner
        co_await when_all<async_task<op1_ret_t>>({ &op1, &op2, &op3 });
#else
        when_all<async_task<int>> wa({ &op1, &op2, &op3 });
        co_await wa;
#endif
        printf("Class01::coroutine1(): result = %d\n", op1.get_result() +  op2.get_result() + op3.get_result());
    } // g++ 11 reports at this line: error: array used as initializer
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01a.coroutine1(); });
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
