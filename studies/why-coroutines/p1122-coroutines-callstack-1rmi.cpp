/**
 * @file p1122-coroutines-callstack-1rmi.cpp
 * @brief
 * Variant of p1120. References to lower layer objects are passed via the constructor.
 * All objects can be and are declared after all class definitions.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"

#include "p1000co.h"

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1Co
 * Upper layer: Layer02 (but not known by Layer01)
 * Note: At the co_await statement:
 *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
 * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
 */
class Layer01
{
public:
    Layer01(RemoteObject1Co& remoteObj1co)
        : m_remoteObj1co(remoteObj1co)
    {}

    async_task<int> coroutine1(int in1, int& out1, int& out2)
    {
        printf("Layer01::coroutine1(in1 = %d, out1 = %d, out2 = %d)\n", in1, out1, out2);
        int ret1 = co_await m_remoteObj1co.op1(in1, in1, out1, out2);
        printf("Layer01::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + ret1;
    }

private:
    RemoteObject1Co& m_remoteObj1co;
};

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 * Note: At the co_await statement:
 *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
 * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
 */
class Layer02
{
public:
    Layer02(Layer01& layer01)
        : m_layer01(layer01)
    {}

    async_task<int> coroutine1(int in1, int& out1)
    {
        printf("Layer01::coroutine1(in1 = %d, out1 = %d)\n", in1, out1);
        int out2 = -1;
        int ret1 = co_await m_layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + out2 + ret1;
    }

private:
    Layer01& m_layer01;
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
        printf("Layer03::coroutine1(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = co_await m_layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine1(): out1 = %d, ret1 = %d\n", out1, ret1);
        co_return in1 + out1 + ret1;
    }

    async_task<int> coroutine2(int in1)
    {
        printf("Layer03::coroutine2(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = co_await m_layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine2(): out1 = %d, ret1 = %d\n", out1, ret1);
        co_return in1 + out1 + ret1;
    }

private:
    Layer02& m_layer02;
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
    printf("main()\n");
    async_task<int> t1 = layer03.coroutine1(2);
    async_task<int> t2 = layer03.coroutine1(3);
    async_task<int> t3 = layer03.coroutine2(2);
    async_task<int> t4 = layer03.coroutine2(3);

    eventQueue.run();
    int ret1 = t1.get_result();
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = t2.get_result();
    printf("main(): ret2 = %d\n", ret2);
    int ret3 = t3.get_result();
    printf("main(): ret3 = %d\n", ret3);
    int ret4 = t4.get_result();
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}
