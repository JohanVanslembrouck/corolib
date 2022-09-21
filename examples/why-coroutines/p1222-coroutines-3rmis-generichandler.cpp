/**
 * @file p1222-coroutines-3rmis-generichandler.cpp
 * @brief Variant of p1220 using a generic version of the completion handler.
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
        print(PRI1, "%p: RemoteObject1Co::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in11, in12);
        return { this, index };
    }

    async_operation<op2_ret_t> start_op2(int in21, int in22)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op2(): index = %d\n", this, index);
        start_op2_impl(index, in21, in22);
        return { this, index };
    }

    async_operation<op1_ret_t> start_op3(int in31)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObject1Co::start_op3(): index = %d\n", this, index);
        start_op3_impl(index, in31);
        return { this, index };
    }

    template<class TYPE>
    void genericCompletionHandler(int idx, TYPE in)
    {
        print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<TYPE>* om_async_operation_t =
            dynamic_cast<async_operation<TYPE>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d): om_async_operation_t->set_result()\n", this, idx);
            om_async_operation_t->set_result(in);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObject1Co::genericCompletionHandler(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    }

    // Lower level functions
    void sendc_op1(int in11, int in12, lambda_op1_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op1(%d, %d, l)\n", in11, in12);
        eventQueue.push([lambda]() { lambda({ 1, 2, 3 }); });
    }

    void sendc_op2(int in11, int in12, lambda_op2_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op2(%d, %d, l)\n", in11, in12);
        eventQueue.push([lambda]() { lambda({ 1, 2 }); });
    }

    void sendc_op3(int in11, lambda_op1_ret_t lambda)
    {
        printf("RemoteObject1Co::sendc_op3(%d, l)\n", in11);
        eventQueue.push([lambda]() { lambda({ 1, 2, 3 }); });
    }

protected:
    // Implementation functions
    void start_op1_impl(const int idx, int in11, int in12);
    void start_op2_impl(const int idx, int in11, int in12);
    void start_op3_impl(const int idx, int in11);

private:
    RemoteObject1 m_remoteObject;
};

void RemoteObject1Co::start_op1_impl(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    sendc_op1(in11, in12,
        [this, idx](op1_ret_t in)
        {
            genericCompletionHandler<op1_ret_t>(idx, in);
        });
}

void RemoteObject1Co::start_op2_impl(const int idx, int in11, int in12)
{
    print(PRI1, "%p: RemoteObject1Co::start_op2_impl(%d)\n", this, idx);

    sendc_op2(in11, in12,
        [this, idx](op2_ret_t in)
        {
            genericCompletionHandler<op2_ret_t>(idx, in);
        });
}

void RemoteObject1Co::start_op3_impl(const int idx, int in11)
{
    print(PRI1, "%p: RemoteObj1::start_op3_impl(%d)\n", this, idx);

    sendc_op3(in11,
        [this, idx](op1_ret_t in)
        {
            genericCompletionHandler<op1_ret_t>(idx, in);
        });
}

RemoteObject1Co remoteObj1co{ remoteObj1 };
RemoteObject1Co remoteObj2co{ remoteObj2 };
RemoteObject1Co remoteObj3co{ remoteObj3 };

class Class01a
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01a::coroutine1()\n");
        op1_ret_t res1 = co_await remoteObj1co.start_op1(gin11, gin12);
        // 1 Do stuff
        if (res1.ret == gval1) {
            op2_ret_t res1 = co_await remoteObj2co.start_op2(gin21, gin22);
            // 2 Do stuff
        }
        else {
            op1_ret_t res3 = co_await remoteObj3co.start_op3(gin31);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        printf("Class01a::coroutine1a()\n");
        async_operation<op1_ret_t> op1 = remoteObj1co.start_op1(gin11, gin12);
        // 1a Do some stuff that doesn't need the result of the RMI
        op1_ret_t res1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (res1.ret == gval1) {
            async_operation<op2_ret_t> op2 = remoteObj2co.start_op2(gin21, gin22);
            // 2a Do some stuff that doesn't need the result of the RMI
            op2_ret_t res1 = co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3co.start_op3(gin31);
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
        int ret1 = co_await remoteObj1co.op1(gin11, gin12, gout11, gout12);
        // 1 Do stuff
        if (ret1 == gval1) {
            int ret2 = co_await remoteObj2co.op2(gin21, gin22, gout21);
            // 2 Do stuff
        }
        else {
            int ret3 = co_await remoteObj3co.op3(gin31, gout31, gout32);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a()
    {
        async_task<int> op1 = remoteObj1co.op1(gin11, gin12, gout11, gout12);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            async_task<int> op2 = remoteObj2co.op2(gin21, gin22, gout21);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_task<int> op3 = remoteObj3co.op3(gin31, gout31, gout32);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
