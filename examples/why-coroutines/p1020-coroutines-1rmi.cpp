/**
 * @file p1020-coroutines-1rmi.cpp
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

protected:
    // Implementation functions
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
    
    eventQueue.push(
            [operation1]() 
            { 
                (*operation1)(1, 2, 3);
                delete operation1;
            });
}

RemoteObj1 remoteObj1;

class Class01a
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1() - part 1\n");
        op1_ret_t ret = co_await remoteObj1.start_op1(gin11, gin12);
        printf("Class01a::coroutine1(): ret.out1 = %d, ret.out2 = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        printf("Class01a::coroutine1() - part 2\n");
    }
    
    async_task<void> coroutine1a()
    {
        printf("Class01a::coroutine1a() - part 1\n");
        async_operation<op1_ret_t> op1 = remoteObj1.start_op1(gin11, gin12);
        op1_ret_t ret = co_await op1;
        printf("Class01a::coroutine1a(): ret.out1 = %d, ret.outé = %d, ret.ret = %d\n", ret.out1, ret.out2, ret.ret);
        printf("Class01a::coroutine1a() - part 2\n");
    }
};

struct Class01
{
    async_task<void> coroutine1()
    {
        printf("Class01::coroutine1() - part 1\n");
        int ret1 = co_await remoteObj1.op1(gin11, gin12, gout11, gout12);
        printf("Class01::coroutine1(): gout11 = %d, gout12 = %d, ret1 = %d\n", gout11, gout12, ret1);
        printf("Class01::coroutine1() - part 2\n");
    }
    
    async_task<void> coroutine1a()
    {
        printf("Class01::coroutine1a() - part 1\n");
        async_task<int> op1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
        int ret1 = co_await op1;
        printf("Class01::coroutine1a(): gout11 = %d, gout12 = %d, ret1 = %d\n", gout11, gout12, ret1);
        printf("Class01::coroutine1a() - part 2\n");
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
