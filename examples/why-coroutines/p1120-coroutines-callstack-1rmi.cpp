/**
 * @file p1120-coroutines-callstack-1rmi.cpp
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
    
    // Start-up function
    async_operation<op1_ret_t> start_op1(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        async_operation<op1_ret_t> ret{ this, index };
        start_op1_impl(index, in11, in12);
        return ret;
    }

    lambda_3int_t operation1;

protected:
    // Implementation function
    void start_op1_impl(const int idx, int in11, int in12);
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
    
    // Because we do not have anyone else to produce the completion event, we do it ourselves
    eventQueue.push(
        [operation1]()
        {
            (*operation1)(1, 2, 3);
            delete operation1;
        });
}

RemoteObj1 remoteObj1;

class Layer01
{
public:
    async_task<int> coroutine1(int in1, int& out11, int& out12)
    {
        printf("Layer01::coroutine1(): part 1\n");
        int ret1 = co_await remoteObj1.op1(in1, in1, out11, out12);
        printf("Layer01::coroutine1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::coroutine1(): part 2\n");
        co_return ret1;
    }
};

Layer01 layer01;

class Layer02
{
public:
    async_task<int> coroutine1(int in1, int& out1)
    {
        printf("Layer02::coroutine1(): part 1\n");
        int ret1 = co_await layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1(): part 2\n");
        co_return ret1;
    }
private:
    int    out2{0};
};

Layer02 layer02;

class Layer03
{
public:
    async_task<int> coroutine1(int in1)
    {
        printf("Layer03::coroutine1(): part 1\n");
        int ret1 = co_await layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine1(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::coroutine1(): part 2\n");
        co_return ret1;
    }
private:
    int    out1{0};
};

Layer03 layer03;

int main()
{
    printf("main();\n");
    connect(event1, []() { layer03.coroutine1(2); });
    connect(event2, []() { layer03.coroutine1(3); });
    eventQueue.run();
    return 0;
}
