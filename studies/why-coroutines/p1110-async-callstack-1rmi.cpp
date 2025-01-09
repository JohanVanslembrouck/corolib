/**
 * @file p1110-async-callstack-1rmi.cpp
 * @brief Asynchronous implementation of p1100-sync-callstack-1rmi.cpp.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

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
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("Layer01::function1_ctxt_t::~function1_ctxt_t()\n"); }

        void* ctxt;
        lambda_vp_3int_t lambda;
        int in1;
    };

public:
    void function1(void* ctxt1, lambda_vp_3int_t lambda, int in1)
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ ctxt1, lambda, in1 };

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
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(context);
        // call function1_cb of upper layer
        // return in1 + ret1;
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
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("Layer02::function1_ctxt_t::~function1_ctxt_t()\n"); }

        void* ctxt;
        lambda_vp_2int_t lambda;
        int in1;
    };

public:
    void function1(void* ctxt1, lambda_vp_2int_t lambda, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ ctxt1, lambda, in1 };

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
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(context);
        // call function1_cb of upper layer
        // return in1 + out2 + ret1;
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
private:
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("Layer03::function1_ctxt_t::~function1_ctxt_t()\n"); }

        int* ret;
        int in1;
    };

public:
    void function1(int in1, int& ret1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ &ret1, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            [this](void *ctxt, int out1, int ret1) {
                this->function1_cb(ctxt, out1, ret1);
            },
            in1);
    }

protected:
    void function1_cb(void* context, int out1, int ret1)
    {
        printf("Layer03::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(context);
        // return in1 + out1 + ret1;
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
    }

private:
    struct function2_ctxt_t
    {
        ~function2_ctxt_t() { printf("Layer03::function2_ctxt_t::~function2_ctxt_t()\n"); }

        int* ret;
        int in1;
    };

public:
    void function2(int in1, int& ret1)
    {
        printf("Layer03::function2(in1 = %d)\n", in1);
        function2_ctxt_t* ctxt = new function2_ctxt_t{ &ret1, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            [this](void* ctxt, int out1, int ret1) {
                this->function2_cb(ctxt, out1, ret1);
            },
            in1);
    }

protected:
    void function2_cb(void* context, int out1, int ret1)
    {
        printf("Layer03::function2_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        function2_ctxt_t* ctxt = static_cast<function2_ctxt_t*>(context);
        // return in1 + out1 + ret1;
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
    }
};

Layer03 layer03;

EventQueue eventQueue;

int main() {
    printf("main()\n");
    int ret1 = -1;
    layer03.function1(2, ret1);
    int ret2 = -1;
    layer03.function1(3, ret2);
    int ret3 = -1;
    layer03.function2(2, ret3);
    int ret4 = -1;
    layer03.function2(3, ret4);

    eventQueue.run();
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}
