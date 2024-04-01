/**
 * @file p1130-coroutines-async-callstack-1rmi.cpp
 * @brief Variant of p1110-async-callstack-1rmi.cpp.
 *
 * In this example a coroutine layer is added on top of the highest asynchronous layer, Layer03.
 * It shows that only a few simple modifications have to be made to Layer03.
 * 
 * This approach can be useful in case it is not possible or desirable to make modifications
 * to an existing asynchronous callstack, yet the application should be able to use coroutines.
 * 
 * This example has the same problem as described in p1110-async-callstack-1rmi.cpp.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "eventqueue.h"
#include "p1000.h"

RemoteObject1 remoteObj1;

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1
 * Upper layer: Layer02 (via m_lambda)
 */
class Layer01
{
public:
    // Synchronous function:
    // int function1(int in11, int& out12, int& out12)
    
    void function1(int in1, lambda_2int_t lambda) 
    {
        printf("Layer01::function1(): part 1\n");
        m_lambda = lambda;
        remoteObj1.sendc_op1(in1, in1, 
            [this](int out1, int out2, int ret1)
            { 
                this->function1_cb(out1, out2, ret1); 
            });
        printf("Layer01::function1(): return\n");
    }

    void function1_cb(int out1, int out2, int ret1) 
    {
        printf("Layer01::function1_cb(%d, %d, %d)\n", out1, out2, ret1);
        printf("Layer01::function1_cb(): part 2\n");
        // call function1_cb of upper layer
        m_lambda(out1, ret1);
    }
private:
    lambda_2int_t m_lambda;
};

Layer01 layer01;

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (via m_lambda)
 */
class Layer02
{
public:
    // Synchronous function:
    // int function1(int in1, int& out11)
    
    void function1(int in1, lambda_1int_t lambda)
    {
        printf("Layer02::function1(): part 1\n");
        m_lambda = lambda;
        layer01.function1(in1, 
            [this](int out1, int ret1) { 
                this->function1_cb(out1, ret1);
            });
        printf("Layer02::function1(): return\n");
    }

    void function1_cb(int out1, int ret1)
    {
        printf("Layer02::function1_cb(%d, %d)\n", out1, ret1);
        printf("Layer02::function1_cb(): part 2\n");
        // call function1_cb of upper layer
        m_lambda(ret1);
    }
private:
    lambda_1int_t m_lambda;
};

Layer02 layer02;

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (not known by Layer03)
 *
 */
class Layer03
{
public:
    // Synchronous function:
    // int function1(int in1);

    void function1(int in1, async_operation<int>& op1)
    {
        printf("Layer03::function1(): part 1\n");
        layer02.function1(in1,
            [this, &op1](int ret1) {  
				op1.set_result(ret1);
                op1.completed();
            });
        printf("Layer03::function1(): return\n");
    }

    // Synchronous function:
    // int function2(int in1);
    
    void function2(int in1, async_operation<int>& op1)
    {
        printf("Layer03::function2(): part 1\n");
        layer02.function1(in1,
            [this, &op1](int ret1) {
                op1.set_result(ret1);
                op1.completed();
            });
    }
};

Layer03 layer03;

class Layer03Co : public CommService
{
public:
    async_task<int> coroutine1(int in1)
    {
        printf("Layer03::coroutine1(): part 1\n");
		int index = get_free_index();
		async_operation<int> op1{this, index};
        layer03.function1(in1, op1);
        printf("Layer03::coroutine1(): int ret1 = co_await op1\n");
		int ret1 = co_await op1;
        printf("Layer03::coroutine1(): ret1 = %d\n", ret1);
        printf("Layer03::coroutine1(): part 2\n");
        co_return in1 + ret1;
    }

    async_task<int> coroutine2(int in1)
    {
        printf("Layer03::coroutine2(): part 1\n");
        int index = get_free_index();
        async_operation<int> op1{ this, index };
        layer03.function2(in1, op1);
        printf("Layer03::coroutine2(): int ret1 = co_await op1\n");
        int ret1 = co_await op1;
        printf("Layer03::coroutine2(): ret1 = %d\n", ret1);
        printf("Layer03::coroutine2(): part 2\n");
        co_return in1 + ret1;
    }

private:
    int    out1{0};
};

Layer03Co layer03co;

EventQueue eventQueue;

int main() {
    printf("main()\n");
    async_task<int> t1 = layer03co.coroutine1(2);
    //async_task<int> t2 = layer03co.coroutine2(3);
    printf("main(): eventQueue.run();\n");
    eventQueue.run();
    int ret1 = t1.get_result();
    printf("main(): ret1 = %d\n", ret1);
    //int ret2 = t2.get_result();
    //printf("main(): ret2 = %d\n", ret2);
    return 0;
}
