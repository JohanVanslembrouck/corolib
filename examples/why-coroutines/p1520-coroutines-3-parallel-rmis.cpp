/**
 * @file p1520-coroutines-3-parallel-rmis.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

using namespace corolib;

class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}
    
    // User API
    async_task<int> op1(int in11, int in12, int& out11, int& out12)
    {
        async_operation<op1_ret_t> op1 = start_op1(in11, in12);
        op1_ret_t res = co_await op1;
        out11 = res.out1;
        out12 = res.out2;
        co_return res.ret;
    }

    // Start-up functions
    async_operation<op1_ret_t> start_op1(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in11, in12);
        return { this, index };
    }

protected:
    // Implementation functions
    void start_op1_impl(const int idx, int in11, int in12);
    
private:
    RemoteObject1 m_remoteObject;
};

void RemoteObject1Co::start_op1_impl(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    m_remoteObject.sendc_op1(in11, in12, 
        [this, idx](int out11, int out12, int ret1)
        {
            print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<op1_ret_t>* om_async_operation_t =
                dynamic_cast<async_operation<op1_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                op1_ret_t op1_ret = { out11, out12, ret1 };
                om_async_operation_t->set_result(op1_ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
         });
}

RemoteObject1Co remoteObj1co{remoteObj1};
RemoteObject1Co remoteObj2co{remoteObj2};
RemoteObject1Co remoteObj3co{remoteObj3};

class Class01a
{
public: 
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1()\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(gin11, gin12);
        async_operation<op1_ret_t> op2 = remoteObj2co.start_op1(gin11, gin12);
        async_operation<op1_ret_t> op3 = remoteObj3co.start_op1(gin11, gin12);
        co_await when_all<async_operation<op1_ret_t>>({ &op1, &op2, &op3 });
        printf("Class01a::coroutine1(); result = %d\n", op1.get_result().ret +  op2.get_result().ret + op3.get_result().ret);
    }
};

Class01a class01a;

class Class01
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01::coroutine1()\n");
        async_task<int> op1 = remoteObj1co.op1(gin11, gin12, gout11, gout12);
        async_task<int> op2 = remoteObj2co.op1(gin11, gin12, gout11, gout12);
        async_task<int> op3 = remoteObj3co.op1(gin11, gin12, gout11, gout12);
        co_await when_all<async_task<int>>({ &op1, &op2, &op3 });
        printf("Class01::coroutine1(): result = %d\n", op1.get_result() +  op2.get_result() + op3.get_result());
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01a.coroutine1(); });
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
