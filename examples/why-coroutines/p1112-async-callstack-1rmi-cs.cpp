/**
 * @file p1112-async-callstack-1rmi-cs.cpp
 * @brief Asynchronous implementation of p1100-sync-callstack-1rmi.cpp.
 *
 * Instead of passing a lambda and a context while calling a function at a lower layer,
 * a higher layer passes a CallStack object to the lower layer.
 * The highest layer creates a CallStack object for every asynchronous function 
 * it calls on its lower layer.
 * The CallStack object is also passed vhen returning from a lower layer to a higher layer,
 *
 * The CallStack contains a stack<void*> data member that stores a stack of lambdas.
 * Different CallStack objects may thus store lambdas that correspond to a different callback
 * function at a given layer.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <stack>

#include "common.h"
#include "eventqueue.h"

class CallStack;

typedef std::function<void(CallStack*, int, int, int)>  lambda_cs_3int_t;
typedef std::function<void(CallStack*, int, int)>       lambda_cs_2int_t;
typedef std::function<void(CallStack*, int)>            lambda_cs_1int_t;
typedef std::function<void(CallStack*)>                 lambda_cs_t;

using namespace std;

struct StackElement
{
    StackElement(void* context_, void* lambda_)
        : context(context_)
        , lambda(lambda_)
    {
        printf("%p: StackElement::StackElement(): context = %p, lambda = %p\n", this, context, lambda);
    }

    ~StackElement()
    {
        printf("%p: StackElement:~StackElement(): context = %p, lambda = %p\n", this, context, lambda);
    }

    void* context;
    void* lambda;
};

/**
 * @brief Callstack allows building a stack of lambdas that is passed
 * along the layers when one layer calls the function of the lower layer.
 * When returning "up the call chain", every layer knows which lambda to use.
 *
 */
class CallStack
{
    stack<StackElement*> q;
public:

    void push(StackElement* val)
    {
        q.push(val);
    }

    void pop()
    {
        if (q.empty())
            printf("No elements\n");
        else
            q.pop();
    }

    StackElement* top()
    {
        return (q.empty()) ? nullptr : q.top();
    }

    StackElement* top_pop()
    {
        StackElement* val = (q.empty()) ? nullptr : q.top();
        q.pop();
        return val;
    }

    bool empty()
    {
        return (q.empty());
    }
};

class RemoteObject1
{
public:
    void sendc_op1(CallStack* callstack, int in1, int in2)
    {
        printf("RemoteObject1::sendc_op1(in1 = %d, in2 = %d)\n", in1, in2);

        StackElement* se = callstack->top();
        lambda_cs_3int_t* op = static_cast<lambda_cs_3int_t*>(se->lambda);

        eventQueue.push(
            [op, callstack, in1, in2]()
            { 
                (*op)(callstack, 1, 2, in1 + in2);
                delete op; 
            });
    }
};

RemoteObject1 remoteObj1;

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1
 * Upper layer: Layer02 (but not known by Layer01)
 *
 */
class Layer01
{
public:
    struct function1_cxt_t
    {
        int in1;
    };

    void function1(CallStack* callstack, int in1) 
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack* callstack, int out1, int out2, int ret1)
            {
                this->function1_cb(callstack, out1, out2, ret1);
            });
        void* ctxt = new function1_cxt_t{ in1 };
        StackElement* se = new StackElement(ctxt, op);
        callstack->push(se);

        remoteObj1.sendc_op1(callstack, in1, in1);
    }

    void function1_cb(CallStack* callstack, int out1, int out2, int ret1) 
    {
        printf("Layer01::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);

        StackElement* se = callstack->top_pop();
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(se->context);

        StackElement* se2 = callstack->top();
        lambda_cs_3int_t* op = static_cast<lambda_cs_3int_t*>(se2->lambda);
        (*op)(callstack, out1, out2, ctxt->in1 + ret1);
        delete op;
        delete ctxt;
        delete se;
    }
};

Layer01 layer01;

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 */
class Layer02
{
public:    
    struct function1_cxt_t
    {
        int in1;
    };
    
    void function1(CallStack* callstack, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);

        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack* callstack, int out1, int out2, int ret1)
            {
                this->function1_cb(callstack, out1, out2, ret1);
            });
        void* ctxt = new function1_cxt_t{ in1 };
        StackElement* se = new StackElement(ctxt, op);
        callstack->push(se);

        layer01.function1(callstack, in1);
    }

    void function1_cb(CallStack* callstack, int out1, int out2, int ret1)
    {
        printf("Layer02::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        StackElement* se = callstack->top_pop();
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(se->context);

        StackElement* se2 = callstack->top();
        lambda_cs_2int_t* op = static_cast<lambda_cs_2int_t*>(se2->lambda);

        (*op)(callstack, 1, ctxt->in1 + out2 + ret1);
        delete op;
        delete ctxt;
        delete se;
    }
};

Layer02 layer02;

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (but not known by Layer03)
 *
 */
class Layer03
{
public:
    struct function1_cxt_t
    {
        int* ret;
        int in1;
    };

    void function1(int in1, int& ret1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        CallStack* callstack = new CallStack;
         
        lambda_cs_2int_t* p = new lambda_cs_2int_t(
            [this](CallStack* callstack, int out1, int ret1)
            {
                this->function1_cb(callstack, out1, ret1);
            });
        void* ctxt = new function1_cxt_t{ &ret1, in1 };
        StackElement* se = new StackElement(ctxt, p);
        callstack->push(se);

        layer02.function1(callstack, in1);
    }

    void function1_cb(CallStack* callstack, int out1, int ret1)
    {
        printf("Layer03::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        StackElement* se = callstack->top_pop();
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(se->context);
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
        delete se;
        delete callstack;
    }

    struct function2_cxt_t
    {
        int* ret;
        int in1;
    };
    
    void function2(int in1, int& ret1)
    {
        printf("Layer03::function2(in1 = %d)\n", in1);
        CallStack* callstack = new CallStack;

        lambda_cs_2int_t* p = new lambda_cs_2int_t(
            [this](CallStack* callstack, int out1, int ret1)
            {
                this->function2_cb(callstack, out1, ret1);
            });
        void* ctxt = new function2_cxt_t{ &ret1, in1 };
        StackElement* se = new StackElement(ctxt, p);
        callstack->push(se);

        layer02.function1(callstack, in1);
    }

    void function2_cb(CallStack* callstack, int out1, int ret1)
    {
        printf("Layer03::function2_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        StackElement* se = callstack->top_pop();
        function2_cxt_t* ctxt = static_cast<function2_cxt_t*>(se->context);
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
        delete se;
        delete callstack;
    }
};

Layer03 layer03;

EventQueue eventQueue;

int main() {
    printf("main()\n");
    int ret1 = -1;
    layer03.function1(2, ret1);
    int ret2 = -1;
    layer03.function2(3, ret2);
    printf("main(): eventQueue.run();\n");
    eventQueue.run();
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
