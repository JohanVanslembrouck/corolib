/**
 * @file p1122-coroutines-callstack-1rmi.cpp
 * @brief
 * Variant of p1120. References to lower layer objects are passed via the constructor.
 * All objects can be and are declared after all class definitions.
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

#include "p1000.h"

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
    
    // Start-up function
    async_operation<op1_ret_t> start_op1(int in11, int in12)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        start_op1_impl(index, in11, in12);
        return { this, index };
    }

protected:
    // Implementation function
    void start_op1_impl(const int idx, int in11, int in12);
    
private:
    RemoteObject1& m_remoteObject;
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

class Layer01
{
public:
    Layer01(RemoteObject1Co& remoteObj1co)
        : m_remoteObj1co(remoteObj1co)
    {}

    async_task<int> coroutine1(int in1, int& out11, int& out12)
    {
        printf("Layer01::coroutine1(): part 1\n");
        int ret1 = co_await m_remoteObj1co.op1(in1, in1, out11, out12);
        printf("Layer01::coroutine1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::coroutine1(): part 2\n");
        co_return ret1;
    }

private:
    RemoteObject1Co& m_remoteObj1co;
};

class Layer02
{
public:
    Layer02(Layer01& layer01)
        : m_layer01(layer01)
    {}

    async_task<int> coroutine1(int in1, int& out1)
    {
        printf("Layer02::coroutine1(): part 1\n");
        int ret1 = co_await m_layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1(): part 2\n");
        co_return ret1;
    }

private:
    Layer01& m_layer01;
    int    out2{0};
};

class Layer03
{
public:
    Layer03(Layer02& layer02)
        : m_layer02(layer02)
    {}

    async_task<int> coroutine1(int in1)
    {
        printf("Layer03::coroutine1(): part 1\n");
        int ret1 = co_await m_layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine1(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::coroutine1(): part 2\n");
        co_return ret1;
    }

    async_task<int> coroutine2(int in1)
    {
        printf("Layer03::coroutine2(): part 1\n");
        int ret1 = co_await m_layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine2(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::coroutine2(): part 2\n");
        co_return ret1;
    }

private:
    Layer02& m_layer02;
    int    out1{0};
};

RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{ remoteObj1 };
Layer01 layer01{ remoteObj1co };
Layer02 layer02{ layer01 };
Layer03 layer03{ layer02 };

int main()
{
    printf("main();\n");
    eventQueue.push([]() { layer03.coroutine1(2); });
    eventQueue.push([]() { layer03.coroutine2(3); });
    eventQueue.run();
    return 0;
}
