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

using namespace corolib;

#include "common.h"
#include "variables.h"

#include "p1000co.h"

/**
 * @brief Layer01 is the lowest level in the application stack
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

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 *
 */
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

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (but not known by Layer03)
 *
 */
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

/** 
 * Object delcarations
 *
 */
//                            lower layer
RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{ remoteObj1 };
Layer01 layer01             { remoteObj1co };
Layer02 layer02             { layer01 };
Layer03 layer03             { layer02 };

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { layer03.coroutine1(2); });
    eventQueue.push([]() { layer03.coroutine2(3); });
    eventQueue.run();
    return 0;
}
