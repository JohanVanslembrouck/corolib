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
private:
    struct function1_cxt_t
    {
        void* ctxt;
        lambda_vp_3int_t lambda;
        int in1;
    };

public:
    void function1(void* ctxt1, lambda_vp_3int_t lambda, int in1)
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ ctxt1, lambda, in1 };

        // int ret1 = remoteObj1.op1(in1, in1, out1, out2);
        remoteObj1.sendc_op1(ctxt, in1, in1,
            [this](void* context, int out1, int out2, int ret1)
            {
                this->function1_cb(context, out1, out2, ret1);
            });
    }

protected:
    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Layer01::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // call function1_cb of upper layer
        //  return in1 + ret1;
        ctxt->lambda(ctxt->ctxt, out1, out2, ctxt->in1 + ret1);
        delete ctxt;
    }
};

Layer01 layer01;

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (via m_lambda)
 */
class Layer02
{
private:
    struct function1_cxt_t
    {
        void* ctxt;
        lambda_vp_2int_t lambda;
        int in1;
    };

public:
    void function1(void* ctxt1, lambda_vp_2int_t lambda, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ ctxt1, lambda, in1 };

        // int ret1 = layer01.function1(in1, out1, out2);
        layer01.function1(
            ctxt,
            [this](void* ctxt1, int out1, int out2, int ret1) {
                this->function1_cb(ctxt1, out1, out2, ret1);
            },
            in1);
    }

protected:
    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Layer02::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // call function1_cb of upper layer
        //return in1 + out2 + ret1;
        ctxt->lambda(ctxt->ctxt, out1, ctxt->in1 + out2 + ret1);
        delete ctxt;
    }
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
    // Synchronous function:
    // int function1(int in1);

    // Alternative 1
private:
    struct function1_cxt_t
    {
        int* ret;
        int in1;
    };

public:
    void function1(int in1, int& ret1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ &ret1, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            [this](void* ctxt, int out1, int ret1) {
                this->function1_cb(ctxt, out1, ret1);
            },
            in1);
    }

protected:
    void function1_cb(void* context, int out1, int ret1)
    {
        printf("Layer03::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        //return in1 + out1 + ret1;
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
    }

public:
    void function1(int in1, lambda_vp_2int_t lambda)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ nullptr, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            lambda,
            in1);
    }

    // Synchronous function:
    // int function2(int in1);

    // Alternative 2
public:
    void function2(async_operation<int>& op1, int in1)
    {
        printf("Layer03::function2(in1 = %d)\n", in1);
        void* ctxt = nullptr;
        layer02.function1(ctxt,
            [this, in1, &op1](void* ctxt, int out1, int ret1) {
                op1.set_result(in1 + out1 + ret1);
                op1.completed();
            },
            in1);
        printf("Layer03::function2(): return\n");
    }
};

Layer03 layer03;

class Layer03Co : public CommService
{
public:
    // Alternative 1

    async_task<int> coroutine1(int in1)
    {
        printf("Layer03Co::coroutine1(in1 = %d)\n", in1);
        async_operation<int> op1 = start_op1(in1);
        printf("Layer03Co::coroutine1(): int ret1 = co_await op1\n");
        int ret1 = co_await op1;
        printf("Layer03Co::coroutine1(): ret1 = %d\n", ret1);
        co_return ret1;
    }

    async_operation<int> start_op1(int in1)
    {
        int index = get_free_index();
        async_operation<int> ret{ this, index };
        start_op1_impl(index, in1);
        return ret;
    }

    void start_op1_impl(int idx, int in1)
    {
        layer03.function1(in1,
            [this, idx, in1](void* ctxt, int out1, int ret1) {
                completionHandler<int>(idx, in1 + out1 + ret1);
            }
            );
    }

    // Alternative 2

    async_task<int> coroutine2(int in1)
    {
        printf("Layer03Co::coroutine2(in1 = %d)\n", in1);
        int index = get_free_index();
        async_operation<int> op1{ this, index };
        layer03.function2(op1, in1);
        printf("Layer03Co::coroutine2(): int ret1 = co_await op1\n");
        int ret1 = co_await op1;
        printf("Layer03Co::coroutine2(): ret1 = %d\n", ret1);
        co_return ret1;
    }
};

Layer03Co layer03co;

EventQueue eventQueue;

int main() {
    printf("main()\n");
    int ret0 = -1;
    layer03.function1(2, ret0);
    async_task<int> t1 = layer03co.coroutine1(2);
    async_task<int> t2 = layer03co.coroutine1(3);
    async_task<int> t3 = layer03co.coroutine2(2);
    async_task<int> t4 = layer03co.coroutine2(3);
    
    eventQueue.run();
    printf("main(): ret0 = %d\n", ret0);
    int ret1 = t1.get_result();
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = t2.get_result();
    printf("main(): ret2 = %d\n", ret2);
    int ret3 = t3.get_result();
    printf("main(): ret3 = %d\n", ret3);
    int ret4 = t4.get_result();
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}
