/**
 * @file p1126-coroutines-callstack-1rmi.cpp
 * @brief
 * Variant of p1124 where each layer has a reference to a lower layer object (as in p1122 and p1124)
 * and to a higher layer object.
 * This allows going downwards the stack (higher layers to lower layers)
 * and upwards the stack (lower layers to higher layers) and in both directions.
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

RemoteObject1 remoteObj1;

class Layer01;
class Layer02;
class Layer03;
class RemoteObject1Co;

class Layer01
{
public:
    Layer01(RemoteObject1Co& remoteObj1co, Layer02& layer02)
        : m_remoteObj1co(remoteObj1co)
        , m_layer02(layer02)
    {}

    virtual async_task<int> coroutine1(int in1, int& out11, int& out12) = 0;
    virtual async_task<int> coroutine1u(int in1, int& out11, int& out12) = 0;

protected:
    RemoteObject1Co& m_remoteObj1co;
    Layer02& m_layer02;
};

class Layer02
{
public:
    Layer02(Layer01& layer01, Layer03& layer03)
        : m_layer01(layer01)
        , m_layer03(layer03)
    {}

    virtual async_task<int> coroutine1(int in1, int& out1) = 0;
    virtual async_task<int> coroutine1u(int in1, int& out1) = 0;

protected:
    Layer01& m_layer01;
    Layer03& m_layer03;
};

class Layer03
{
public:
    Layer03(Layer02& layer02)
        : m_layer02(layer02)
    {}

    virtual async_task<int> coroutine1(int in1) = 0;
    virtual async_task<int> coroutine1u(int in1, int& out11, int& out12) = 0;

protected:
    Layer02& m_layer02;
};


class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject, Layer01& layer01)
        : m_remoteObject(remoteObject)
        , m_layer01(layer01)
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

    async_task<int> start_op1u(int in11, int in12)
    {
        print(PRI1, "RemoteObj1::start_op1u(): begin\n");
        int ret1 = co_await m_layer01.coroutine1u(in11, in12, in12);
        print(PRI1, "RemoteObj1::start_op1u(): end\n");
        co_return ret1;
    }

protected:
    // Implementation function
    void start_op1_impl(const int idx, int in11, int in12);
    
private:
    RemoteObject1 m_remoteObject;
    Layer01& m_layer01;
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


class Layer01d : public Layer01
{
public:
    Layer01d(RemoteObject1Co& remoteObj1co, Layer02& layer02)
        : Layer01(remoteObj1co, layer02)
    {}

    async_task<int> coroutine1(int in1, int& out11, int& out12) override
    {
        printf("Layer01::coroutine1(): part 1\n");
        int ret1 = co_await m_remoteObj1co.op1(in1, in1, out11, out12);
        printf("Layer01::coroutine1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::coroutine1(): part 2\n");
        co_return ret1;
    }

    async_task<int> coroutine1u(int in1, int& out11, int& out12) override
    {
        printf("Layer01::coroutine1u(): part 1\n");
        int ret1 = co_await m_layer02.coroutine1u(in1, out11);
        printf("Layer01::coroutine1u(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::coroutine1u(): part 2\n");
        co_return ret1;
    }
};

class Layer02d : public Layer02
{
public:
    Layer02d(Layer01& layer01, Layer03& layer03)
        : Layer02(layer01, layer03)
    {}

    async_task<int> coroutine1(int in1, int& out1) override
    {
        printf("Layer02::coroutine1(): part 1\n");
        int ret1 = co_await m_layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1(): part 2\n");
        co_return ret1;
    }

    async_task<int> coroutine1u(int in1, int& out1) override
    {
        printf("Layer02::coroutine1u(): part 1\n");
        int ret1 = co_await m_layer03.coroutine1u(in1, out1, out2);
        printf("Layer02::coroutine1u(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1u(): part 2\n");
        co_return ret1;
    }

private:
    int    out2{0};
};

class Layer03d : public Layer03
{
public:
    Layer03d(Layer02& layer02)
        : Layer03(layer02)
    {}

    async_task<int> coroutine1(int in1) override
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

    async_task<int> coroutine1u(int in1, int& out11, int& out12) override
    {
        printf("Layer03::coroutine1u(): part 1\n");
        int ret1 = co_await coroutine1(in1);
        printf("Layer03::coroutine1u(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer03::coroutine1u(): part 2\n");
        co_return ret1;
    }

private:
    int    out1{0};
};

extern Layer01d layer01;
extern Layer02d layer02;
extern Layer03d layer03;

RemoteObject1Co remoteObj1co{ remoteObj1, layer01 };
Layer01d layer01{ remoteObj1co, layer02 };
Layer02d layer02{ layer01, layer03 };
Layer03d layer03{ layer02 };

int main()
{
    printf("main();\n");
    eventQueue.push([]() { layer03.coroutine1(2); });
    eventQueue.push([]() { remoteObj1co.start_op1u(3, 4); });
    eventQueue.run();
    return 0;
}
