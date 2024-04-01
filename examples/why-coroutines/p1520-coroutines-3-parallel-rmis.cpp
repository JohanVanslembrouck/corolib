/**
 * @file p1520-coroutines-3-parallel-rmis.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#include "common.h"
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
    async_task<void> coroutine1(int in1, int in2)
    {
        printf("Class01a::coroutine1()\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(in1, in2);
        async_operation<op1_ret_t> op2 = remoteObj2co.start_op1(in1, in2);
        async_operation<op1_ret_t> op3 = remoteObj3co.start_op1(in1, in2);
#if 0
        // The following statement does not compile with g++ 11.3.0
        co_await when_all({ &op1, &op2, &op3 });
#else
        co_await when_all(op1, op2, op3);
#endif
        printf("Class01a::coroutine1(); result = %d\n", op1.get_result().ret +  op2.get_result().ret + op3.get_result().ret);
    } // g++ 11 reports at this line: error: array used as initializer
};

Class01a class01a;

class Class01
{
public:
    async_task<void> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;

        async_task<int> op1 = remoteObj1co.op1(in1, in2, out11, out12);
        async_task<int> op2 = remoteObj2co.op1(in1, in2, out21, out21);
        async_task<int> op3 = remoteObj3co.op1(in1, in2, out31, out32);
#if 0
        // g++ 11 does not like this one-liner
        co_await when_all({ &op1, &op2, &op3 });
#else
        when_all wa({ &op1, &op2, &op3 });
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
    async_task<void> t1 = class01a.coroutine1(11, 12);
    async_task<void> t2 = class01.coroutine1(21, 22);
    eventQueue.run();
    return 0;
}
