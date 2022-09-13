/**
 * @file p1220-coroutines-3rmis.cpp
 * @brief
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

using namespace corolib;

struct op1_ret_t
{
    int out1;
    int out2;
    int ret;
};

struct op2_ret_t
{
    int out1;
    int ret;
};

// -----------------------------------------------------------------------------

class RemoteObj1 : public CommService
{
public:
    // User API
    async_task<int> op1(int in11, int in12, int& out11, int& out12)
    {
        async_operation<op1_ret_t> op1 = start_op1(in11, in12);
        op1_ret_t res = co_await op1;
        out11 = res.out1;
        out12 = res.out2;
        co_return res.ret;
    }
    
    async_task<int> op2(int in21, int in22, int& out21)
    {
        async_operation<op2_ret_t> op2 = start_op2(in21, in22);
        op2_ret_t res = co_await op2;
        out21 = res.out1;
        co_return res.ret;
    }
    
    async_task<int> op3(int in11, int& out31, int& out32)
    {
        async_operation<op1_ret_t> op3 = start_op3(in11);
        op1_ret_t res = co_await op3;
        out31 = res.out1;
        out32 = res.out2;
        co_return res.ret;
    }
    
    // Start-up functions
    async_operation<op1_ret_t> start_op1(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op1_impl(index, in11, in12);
        return ret;
    }
    async_operation<op2_ret_t> start_op2(int in21, int in22)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op2(): index = %d\n", this, index);
        async_operation<op2_ret_t> ret{ this, index };
        start_op2_impl(index, in21, in22);
        return ret;
    }
    async_operation<op1_ret_t> start_op3(int in31)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op3(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op3_impl(index, in31);
        return ret;
    }

protected:
    // Implementation functions
    void start_op1_impl(const int idx, int in11, int in12);
    void start_op2_impl(const int idx, int in11, int in12);
    void start_op3_impl(const int idx, int in11);
};

void RemoteObj1::start_op1_impl(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

    lambda_3int_t* operation1 = new lambda_3int_t(
        [this, idx](int out11, int out12, int ret1)
        {
            print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<op1_ret_t>* om_async_operation_t =
                dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                op1_ret_t op1_ret = { out11, out12, ret1 };
                om_async_operation_t->set_result(op1_ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
    
    eventQueue.push(
        [operation1]() 
        { 
            (*operation1)(1, 2, 3); 
            delete operation1;
        });
}

void RemoteObj1::start_op2_impl(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObj1::start_op2_impl(%d)\n", this, idx);

    lambda_2int_t* operation2 = new lambda_2int_t(
        [this, idx](int out21, int ret2)
        {
            print(PRI1, "%p: RemoteObj1::start_op2_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<op2_ret_t>* om_async_operation_t =
                dynamic_cast<async_operation<op2_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObj1::start_op2_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                op2_ret_t op2_ret = { out21, ret2 };
                om_async_operation_t->set_result(op2_ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObj1::start_op2_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
    
    eventQueue.push(
        [operation2]() 
        { 
            (*operation2)(2, 3); 
            delete operation2;
        });
}

void RemoteObj1::start_op3_impl(const int idx, int in11)
{
    print(PRI1, "%p: RemoteObj1::start_op3_impl(%d)\n", this, idx);

    lambda_3int_t* operation3 = new lambda_3int_t(
        [this, idx](int out31, int out32, int ret3)
        {
            print(PRI1, "%p: Class01::start_op3_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<op1_ret_t>* om_async_operation_t =
                dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: Class01::start_op3_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                op1_ret_t op1_ret = { out31, out32, ret3 };
                om_async_operation_t->set_result(op1_ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObj1::start_op3_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
    
    eventQueue.push(
        [operation3]()
        {
            (*operation3)(1, 2, 3); 
            delete operation3;
        });
}

RemoteObj1 remoteObj1;
RemoteObj1 remoteObj2;
RemoteObj1 remoteObj3;

class Class01a
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1()\n");
        op1_ret_t res1 = co_await remoteObj1.start_op1(gin11, gin12);
        // 1 Do stuff
        if (res1.ret == gval1) {
            op2_ret_t res1 = co_await remoteObj2.start_op2(gin21, gin22);
            // 2 Do stuff
        }
        else {
            op1_ret_t res3 = co_await remoteObj3.start_op3(gin31);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        printf("Class01a::coroutine1a()\n");
        async_operation<op1_ret_t> op1 = remoteObj1.start_op1(gin11, gin12);
        // 1a Do some stuff that doesn't need the result of the RMI
        op1_ret_t res1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (res1.ret == gval1) {
            async_operation<op2_ret_t> op2 = remoteObj2.start_op2(gin21, gin22);
            // 2a Do some stuff that doesn't need the result of the RMI
            op2_ret_t res1 = co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3.start_op3(gin31);
            // 3a Do some stuff that doesn't need the result of the RMI
            op1_ret_t res3 = co_await op3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

struct Class01
{
    async_task<void> coroutine1()
    {
        int ret1 = co_await remoteObj1.op1(gin11, gin12, gout11, gout12);
        // 1 Do stuff
        if (ret1 == gval1) {
            int ret2 = co_await remoteObj2.op2(gin21, gin22, gout21);
            // 2 Do stuff
        }
        else {
            int ret3 = co_await remoteObj3.op3(gin31, gout31, gout32);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        async_task<int> op1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            async_task<int> op2 = remoteObj2.op2(gin21, gin22, gout21);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_task<int> op3 = remoteObj3.op3(gin31, gout31, gout32);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    connect(event1, []() { class01.coroutine1(); });
    connect(event2, []() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
