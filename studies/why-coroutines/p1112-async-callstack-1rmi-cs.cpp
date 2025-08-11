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

using lambda_cs_3int_t = std::function<void(CallStack*, int, int, int)>;
using lambda_cs_2int_t = std::function<void(CallStack*, int, int)>;
using lambda_cs_1int_t = std::function<void(CallStack*, int)>;
using lambda_cs_t = std::function<void(CallStack*)>;

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
private:
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

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1
 * Upper layer: Layer02 (but not known by Layer01)
 *
 */
class Layer01
{
private:
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("Layer01::function1_ctxt_t::~function1_ctxt_t()\n"); }

        int in1;
    };

public:
    void function1(CallStack* callstack, int in1) 
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack* callstack, int out1, int out2, int ret1) {
                this->function1_cb(callstack, out1, out2, ret1);
            });
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1 };
        StackElement* se = new StackElement(ctxt, op);
        callstack->push(se);

        remoteObj1.sendc_op1(callstack, in1, in1);
    }

protected:
    void function1_cb(CallStack* callstack, int out1, int out2, int ret1) 
    {
        printf("Layer01::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);

        StackElement* se = callstack->top_pop();
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(se->context);

        StackElement* se2 = callstack->top();
        lambda_cs_3int_t* op = static_cast<lambda_cs_3int_t*>(se2->lambda);
        (*op)(callstack, out1, out2, ctxt->in1 + ret1);
        delete op;
        delete ctxt;
        delete se;
    }

private:
    RemoteObject1 remoteObj1;
};

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 */
class Layer02
{
private:    
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("Layer02::function1_ctxt_t::~function1_ctxt_t()\n"); }

        int in1;
    };
    
public:
    void function1(CallStack* callstack, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);

        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack* callstack, int out1, int out2, int ret1) {
                this->function1_cb(callstack, out1, out2, ret1);
            });
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1 };
        StackElement* se = new StackElement(ctxt, op);
        callstack->push(se);

        layer01.function1(callstack, in1);
    }

protected:
    void function1_cb(CallStack* callstack, int out1, int out2, int ret1)
    {
        printf("Layer02::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        StackElement* se = callstack->top_pop();
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(se->context);

        StackElement* se2 = callstack->top();
        lambda_cs_2int_t* op = static_cast<lambda_cs_2int_t*>(se2->lambda);

        (*op)(callstack, 1, ctxt->in1 + out2 + ret1);
        delete op;
        delete ctxt;
        delete se;
    }

private:
    Layer01 layer01;
};

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (but not known by Layer03)
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
        CallStack* callstack = new CallStack;
         
        lambda_cs_2int_t* p = new lambda_cs_2int_t(
            [this](CallStack* callstack, int out1, int ret1) {
                this->function1_cb(callstack, out1, ret1);
            });
        function1_ctxt_t* ctxt = new function1_ctxt_t{ &ret1, in1 };
        StackElement* se = new StackElement(ctxt, p);
        callstack->push(se);

        layer02.function1(callstack, in1);
    }

protected:
    void function1_cb(CallStack* callstack, int out1, int ret1)
    {
        printf("Layer03::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        StackElement* se = callstack->top_pop();
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(se->context);
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
        delete se;
        delete callstack;
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
        CallStack* callstack = new CallStack;

        lambda_cs_2int_t* p = new lambda_cs_2int_t(
            [this](CallStack* callstack, int out1, int ret1) {
                this->function2_cb(callstack, out1, ret1);
            });
        function2_ctxt_t* ctxt = new function2_ctxt_t{ &ret1, in1 };
        StackElement* se = new StackElement(ctxt, p);
        callstack->push(se);

        layer02.function1(callstack, in1);
    }

protected:
    void function2_cb(CallStack* callstack, int out1, int ret1)
    {
        printf("Layer03::function2_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        StackElement* se = callstack->top_pop();
        function2_ctxt_t* ctxt = static_cast<function2_ctxt_t*>(se->context);
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
        delete se;
        delete callstack;
    }

private:
    Layer02 layer02;
};

EventQueue eventQueue;

int main() {
    printf("main()\n");
    Layer03 layer03;
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
