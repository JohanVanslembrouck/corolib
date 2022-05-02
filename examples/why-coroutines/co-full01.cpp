/**
 *  Filename: co-full01.cpp
 *  Description:
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/wait_all_awaitable.h>

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
    async_operation<op1_ret_t> start_op1(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op1a(index, in11, in12);
        return ret;
    }
    async_operation<op2_ret_t> start_op2(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op2(): index = %d\n", this, index);
        async_operation<op2_ret_t> ret{ this, index };
        start_op2a(index, in11, in12);
        return ret;
    }
    async_operation<op1_ret_t> start_op3(int in11)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op3(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op3a(index, in11);
        return ret;
    }

    lambda1 operation1;
    lambda2 operation2;
    lambda1 operation3;

protected:
    void start_op1a(const int idx, int in11, int in12);
    void start_op2a(const int idx, int in11, int in12);
    void start_op3a(const int idx, int in11);
};

void RemoteObj1::start_op1a(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObj1::start_op1a(%d)\n", this, idx);

    operation1 = [this, idx](int out11, int out12, int ret1)
    {
        print(PRI1, "%p: RemoteObj1::start_op1a(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<op1_ret_t>* om_async_operation_t =
            dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObj1::start_op1a(%d): om_async_operation_t->set_result()\n", this, idx);
            op1_ret_t op1_ret = { out11, out12, ret1 };
            om_async_operation_t->set_result(op1_ret);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObj1::start_op1a(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    eventQueue.push([this]() { this->operation1(1, 2, 3); });
}

void RemoteObj1::start_op2a(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObj1::start_op2a(%d)\n", this, idx);

    operation2 = [this, idx](int out21, int ret2)
    {
        print(PRI1, "%p: RemoteObj1::start_op2a(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<op2_ret_t>* om_async_operation_t =
            dynamic_cast<async_operation<op2_ret_t>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObj1::start_op2a(%d): om_async_operation_t->set_result()\n", this, idx);
            op2_ret_t op2_ret = { out21, ret2 };
            om_async_operation_t->set_result(op2_ret);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObj1::start_op1a(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    eventQueue.push( [this]() { this->operation2(2, 3); });
}

void RemoteObj1::start_op3a(const int idx, int in11)
{
    print(PRI1, "%p: RemoteObj1::start_op3a(%d)\n", this, idx);

    operation3 = [this, idx](int out31, int out32, int ret3)
    {
        print(PRI1, "%p: Class01::start_op3a(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<op1_ret_t>* om_async_operation_t =
            dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: Class01::start_op3a(%d): om_async_operation_t->set_result()\n", this, idx);
            op1_ret_t op1_ret = { out31, out32, ret3 };
            om_async_operation_t->set_result(op1_ret);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObj1::start_op3a(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };
    
    eventQueue.push([this]() { this->operation3(1, 2, 3); });
}

RemoteObj1 remoteObj1;
RemoteObj1 remoteObj2;
RemoteObj1 remoteObj3;

class Class01
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01::coroutine1()\n");
        async_operation<op1_ret_t> op1 = remoteObj1.start_op1(in11, in12);
        // 1a Do some stuff that doesn't need the result of the RMI
        co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            async_operation<op2_ret_t> op2 = remoteObj2.start_op2(in21, in22);
            // 2a Do some stuff that doesn't need the result of the RMI
            co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3.start_op3(in31);
            // 3a Do some stuff that doesn't need the result of the RMI
            co_await op3;
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
