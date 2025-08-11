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

class Class01a
{
public: 
    async_task<int> coroutine1(int in1, int in2)
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
        int result = op1.get_result().ret + op2.get_result().ret + op3.get_result().ret;
        printf("Class01a::coroutine1(); result = %d\n", result);
        co_return result;
    } // g++ 11 reports at this line: error: array used as initializer

private:
    RemoteObject1 remoteObj1;
    RemoteObject1 remoteObj2;
    RemoteObject1 remoteObj3;

    RemoteObject1Co remoteObj1co{ remoteObj1 };
    RemoteObject1Co remoteObj2co{ remoteObj2 };
    RemoteObject1Co remoteObj3co{ remoteObj3 };
};

class Class01
{
public:
    async_task<int> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;

        async_task<int> op1 = remoteObj1co.op1(in1, in2, out11, out12);
        async_task<int> op2 = remoteObj2co.op1(in1, in2, out21, out22);
        async_task<int> op3 = remoteObj3co.op1(in1, in2, out31, out32);
#if 0
        // The following statement does not compile with g++ 11.3.0
        co_await when_all({ &op1, &op2, &op3 });
#else
        when_all wa(op1, op2, op3);
        co_await wa;
#endif
        int result = op1.get_result() + op2.get_result() + op3.get_result();
        printf("Class01::coroutine1(): result = %d\n", result);
        co_return result;
    } // g++ 11 reports at this line: error: array used as initializer

private:
    RemoteObject1 remoteObj1;
    RemoteObject1 remoteObj2;
    RemoteObject1 remoteObj3;

    RemoteObject1Co remoteObj1co{ remoteObj1 };
    RemoteObject1Co remoteObj2co{ remoteObj2 };
    RemoteObject1Co remoteObj3co{ remoteObj3 };
};

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    Class01a class01a;
    Class01 class01;

    async_task<int> t1a = class01a.coroutine1(11, 12);
    async_task<int> t2a = class01a.coroutine1(21, 22);
    async_task<int> t3a = class01a.coroutine1(31, 32);
    async_task<int> t4a = class01a.coroutine1(41, 42);

    async_task<int> t1 = class01.coroutine1(11, 12);
    async_task<int> t2 = class01.coroutine1(21, 22);
    async_task<int> t3 = class01.coroutine1(31, 32);
    async_task<int> t4 = class01.coroutine1(41, 42);

    eventQueue.run();

    int ret1a = t1a.get_result();
    int ret2a = t2a.get_result();
    int ret3a = t3a.get_result();
    int ret4a = t4a.get_result();

    int ret1 = t1.get_result();
    int ret2 = t2.get_result();
    int ret3 = t3.get_result();
    int ret4 = t4.get_result();

    printf("\n");
    printf("main(): ret1a = %d\n", ret1a);
    printf("main(): ret2a = %d\n", ret2a);
    printf("main(): ret3a = %d\n", ret3a);
    printf("main(): ret4a = %d\n", ret4a);

    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}
