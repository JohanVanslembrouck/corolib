/**
 * @file p1124-coroutines-callstack-1rmi.cpp
 * @brief
 * Variant of p1122 that uses a base class with virtual coroutines for each layer.
 * This approach allows providing and using different implementations for each of the layers.
 * (At the moment only one is provided.)
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1000co.h"

/**
 * Layer base classes and derived classes
 *
 */


/**
 * @brief Layer01 is the lowest level in the application stack.
 * Lower layer: RemoteObject1Co
 * Upper layer: Layer02 (but not known by Layer01)
 *
 */
class Layer01
{
public:
    Layer01(RemoteObject1Co& remoteObj1co)
        : m_remoteObj1co(remoteObj1co)
    {}

    virtual async_task<int> coroutine1(int in1, int& out11, int& out12) = 0;

protected:
    RemoteObject1Co& m_remoteObj1co;
};

class Layer01d : public Layer01
{
public:
    Layer01d(RemoteObject1Co& layer01)
        : Layer01(layer01)
    {}

    async_task<int> coroutine1(int in1, int& out11, int& out12) override
    {
        printf("Layer01::coroutine1(): part 1\n");
        int ret1 = co_await m_remoteObj1co.op1(in1, in1, out11, out12);
        printf("Layer01::coroutine1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::coroutine1(): part 2\n");
        co_return ret1;
    }
};

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 */
class Layer02
{
public:
    Layer02(Layer01& layer01)
        : m_layer01(layer01)
    {}

    virtual async_task<int> coroutine1(int in1, int& out1) = 0;

protected:
    Layer01& m_layer01;
};

class Layer02d : public Layer02
{
public:
    Layer02d(Layer01& layer01)
        : Layer02(layer01)
    {}

    async_task<int> coroutine1(int in1, int& out1) override
    {
        printf("Layer02::coroutine1(): part 1\n");
        int ret1 = co_await m_layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1(): part 2\n");
        co_return ret1;
    }

private:
    int    out2{0};
};

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (but not known by Layer03)
 */
class Layer03
{
public:
    Layer03(Layer02& layer02)
        : m_layer02(layer02)
    {}

    virtual async_task<int> coroutine1(int in1) = 0;

protected:
    Layer02& m_layer02;
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

private:
    int    out1{0};
};

/**
 * Object declaration section
 *
 */
//                            lower layer
RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{ remoteObj1 };
Layer01d layer01            { remoteObj1co };
Layer02d layer02            { layer01 };
Layer03d layer03            { layer02 };

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { layer03.coroutine1(2); });
    eventQueue.push([]() { layer03.coroutine2(3); });
    eventQueue.run();
    return 0;
}
