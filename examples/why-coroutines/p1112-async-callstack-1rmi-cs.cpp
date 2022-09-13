/**
 * @file p1112-async-callstack-1rmi-cs.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <stack>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

// -------------------------------------------------

class CallStack;

typedef std::function<void(CallStack&, int, int, int)>  lambda_cs_3int_t;
typedef std::function<void(CallStack&, int, int)>       lambda_cs_2int_t;
typedef std::function<void(CallStack&, int)>            lambda_cs_1int_t;
typedef std::function<void(CallStack&)>                 lambda_cs_t;

using namespace std;

class CallStack
{
    stack<void*> q;
public:

    void push(void* val)
    {
        printf("Callstack::push(): ptr = %p\n", val);
        q.push(val);
    }

    void pop()
    {
        if (q.empty())
            printf("No elements\n");
        else
            q.pop();
    }

    void* top()
    {
        return (q.empty()) ? nullptr : q.top();
    }

    void* top_pop()
    {
        void* ptr = (q.empty()) ? nullptr : q.top();
        q.pop();
        printf("Callstack::top_pop():  ptr = %p\n", ptr);
        return ptr;
    }

    bool empty()
    {
        return (q.empty());
    }
};


// -------------------------------------------------

class RemoteObject1
{
public:
    void sendc_op1(CallStack& callstack, int in11, int in12)
    {
        printf("RemoteObject1::sendc_op1(callstack %d, %d)\n", in11, in12);
        lambda_cs_3int_t* op = static_cast<lambda_cs_3int_t*>(callstack.top_pop());
        eventQueue.push(
            [op, &callstack]() { 
                (*op)(callstack, 1, 2, 3);
                printf("RemoteObject1::sendc_op1(): delete %p\n", op);
                delete op; 
            });
    }
};

RemoteObject1 remoteObj1;

// -------------------------------------------------

class Layer01
{
public:
    // int function1(int in11, int& out12, int& out12)
    
    void function1(CallStack& callstack, int in1) 
    {
        printf("Layer01::function1(): part 1\n");
        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack& callstack, int out1, int out12, int ret1) {
                this->function1_cb(callstack, out1, out12, ret1);
            });
        callstack.push(op);
        remoteObj1.sendc_op1(callstack, in1, in1);
    }

    void function1_cb(CallStack& callstack, int out11, int out12, int ret1) 
    {
        printf("Layer01::function1_cb(%d, %d, %d)\n", out11, out12, ret1);
        printf("Layer01::function1_cb(): part 2\n");
        // call function1_cb of upper layer (Layer02)
        lambda_cs_2int_t* op = static_cast<lambda_cs_2int_t*>(callstack.top_pop());
        (*op)(callstack, out11, ret1);
        printf("Layer01::function1_cb(): part 2: delete %p\n", op);
        delete op;
    }
};

Layer01 layer01;

// -------------------------------------------------

class Layer02
{
public:    
    // int function1(int in1, int& out11)
    
    void function1(CallStack& callstack, int in1)
    {
        printf("Layer02::function1(): part 1\n");
        lambda_cs_2int_t* op = new lambda_cs_2int_t(
            [this](CallStack& callstack, int out1, int ret1) {
                this->function1_cb(callstack, out1, ret1);
            });
        callstack.push(op);
        layer01.function1(callstack, in1);
    }

    void function1_cb(CallStack& callstack, int out11, int ret1)
    {
        printf("Layer02::function1_cb(%d, %d)\n", out11, ret1);
        printf("Layer02::function1_cb(): part 2\n");
        // call function1_cb of upper layer (Layer03)
        lambda_cs_1int_t* op = static_cast<lambda_cs_1int_t*>(callstack.top_pop());
        (*op)(callstack, ret1);
        printf("Layer02::function1_cb(): part 2: delete %p\n", op);
        delete op;
    }
};

Layer02 layer02;

// -------------------------------------------------

class Layer03
{
public:
    // int function1(int in1);
            
    void function1(int in1)
    {
        printf("Layer03::function1(): part 1\n");
        lambda_cs_1int_t* p = new lambda_cs_1int_t(
            [this](CallStack& callstack, int ret1) {
                this->function1_cb(callstack, ret1);
            });
        m_callstack.push(p);
        layer02.function1(m_callstack, in1);
    }

    void function1_cb(CallStack&, int ret1)
    {
        printf("Layer03::function1_cb(%d)\n", ret1);
        printf("Layer03::function1_cb(): part 2\n");
    }
    
private:
    CallStack m_callstack;
};

Layer03 layer03;

// -------------------------------------------------

int main() {
    printf("main();\n");
    connect(event1, []() { layer03.function1(2); });
    connect(event2, []() { layer03.function1(3); });
    eventQueue.run();
    return 0;
}
