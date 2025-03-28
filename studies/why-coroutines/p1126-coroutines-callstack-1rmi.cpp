/**
 * @file p1126-coroutines-callstack-1rmi.cpp
 * @brief
 * Variant of p1124 where each layer has a reference to a lower layer object (as in p1122 and p1124)
 * but also to a higher layer object.
 * This allows going downwards the stack (higher layer objects call coroutines of lower layers)
 * and upwards the stack (lower layer objects call coroutines of higher layers).
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"
#include "p1000.h"

using namespace corolib;

class Layer01;
class Layer02;
class Layer03;
class RemoteObject1Co;

/**
 * Interfaces (base classes) section
 *
 */

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1Co
 * Upper layer: Layer02
 *
 */
class Layer01
{
public:
    Layer01(RemoteObject1Co& remoteObj1co, Layer02& layer02)
        : m_remoteObj1co(remoteObj1co)
        , m_layer02(layer02)
    {}

    /**
     * @brief "downstream" function: called from upper layer in the application stack
     * @param in1
     * @param out2
     * @param out2
     */
    virtual async_task<int> coroutine1d(int in1, int& out1, int& out2) = 0;
    
    /**
     * @brief "upstream" function: called from lower layer in the protocol stack
     * @param in1
     * @param out2
     * @param out2
     */
    virtual async_task<int> coroutine1u(int in1, int& out1, int& out2) = 0;

protected:
    RemoteObject1Co& m_remoteObj1co;
    Layer02& m_layer02;
};

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03
 *
 */
class Layer02
{
public:
    Layer02(Layer01& layer01, Layer03& layer03)
        : m_layer01(layer01)
        , m_layer03(layer03)
    {}

    virtual async_task<int> coroutine1d(int in1, int& out1) = 0;
    virtual async_task<int> coroutine1u(int in1, int& out1) = 0;

protected:
    Layer01& m_layer01;
    Layer03& m_layer03;
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

    virtual async_task<int> coroutine1d(int in1) = 0;
    virtual async_task<int> coroutine1u(int in1, int& out1, int& out2) = 0;

protected:
    Layer02& m_layer02;
};

/**
 * @brief
 *
 */
class RemoteObject1Co : public CommService
{
public:
    RemoteObject1Co(RemoteObject1& remoteObject, Layer01& layer01)
        : m_remoteObject(remoteObject)
        , m_layer01(layer01)
    {}
    
    // User API
    // --------
    
    async_task<int> op1d(int in1, int in2, int& out1, int& out2)
    {
        async_operation<op1_ret_t> op1 = start_op1d(in1, in2);
        op1_ret_t res = co_await op1;
        out1 = res.out1;
        out2 = res.out2;
        co_return in1 + in2 + res.ret;
    }
    
    // Start functions
    // ---------------
    
    async_operation<op1_ret_t> start_op1d(int in1, int in2)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1d(): index = %d\n", this, index);
        start_op1d_impl(index, in1, in2);
        return { this, index };
    }

    async_task<int> start_op1u(int in1, int in2)
    {
        print(PRI1, "RemoteObj1::start_op1u(): begin\n");
        int ret1 = co_await m_layer01.coroutine1u(in1, in2, in2);
        print(PRI1, "RemoteObj1::start_op1u(): end\n");
        co_return in1 + in2 + ret1;
    }

protected:
    // Implementation function
    // -----------------------
    void start_op1d_impl(const int idx, int in1, int in2);
    
private:
    RemoteObject1 m_remoteObject;
    Layer01& m_layer01;
};

void RemoteObject1Co::start_op1d_impl(const int idx, int in1, int in2)
{
    print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    m_remoteObject.sendc_op1(in1, in2, 
        [this, idx](int out1, int out2, int ret1) 
        {
            print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<op1_ret_t>* om_async_operation_t =
                static_cast<async_operation<op1_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                op1_ret_t op1_ret = { out1, out2, ret1 };
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

/**
 * Derived classes section
 *
 */
 
/**
 * @brief Layer1 derived class
 *
 */
class Layer01d : public Layer01
{
public:
    Layer01d(RemoteObject1Co& remoteObj1co, Layer02& layer02)
        : Layer01(remoteObj1co, layer02)
    {}

    /**
     * Note: At the co_await statement:
     *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
     * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
     */
    async_task<int> coroutine1d(int in1, int& out1, int& out2) override
    {
        printf("Layer01::coroutine1d(): part 1\n");
        int ret1 = co_await m_remoteObj1co.op1d(in1, in1, out1, out2);
        printf("Layer01::coroutine1d(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer01::coroutine1d(): part 2\n");
        co_return in1 + ret1;
    }

    /**
     * Note: At the co_await statement:
     *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
     * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
     */
    async_task<int> coroutine1u(int in1, int& out1, int& out2) override
    {
        printf("Layer01::coroutine1u(): part 1\n");
        int ret1 = co_await m_layer02.coroutine1u(in1, out1);
        printf("Layer01::coroutine1u(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer01::coroutine1u(): part 2\n");
        co_return in1 + ret1;
    }
};

class Layer02d : public Layer02
{
public:
    Layer02d(Layer01& layer01, Layer03& layer03)
        : Layer02(layer01, layer03)
    {}

    /**
     * Note: At the co_await statement:
     *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
     * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
     */
    async_task<int> coroutine1d(int in1, int& out1) override
    {
        printf("Layer02::coroutine1d(): part 1\n");
        int out2 = -1;
        int ret1 = co_await m_layer01.coroutine1d(in1, out1, out2);
        printf("Layer02::coroutine1d(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1d(): part 2\n");
        co_return in1 + out2 + ret1;
    }

    /**
     * Note: At the co_await statement:
     *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
     * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
     */
    async_task<int> coroutine1u(int in1, int& out1) override
    {
        printf("Layer02::coroutine1u(): part 1\n");
        int out2 = -1;
        int ret1 = co_await m_layer03.coroutine1u(in1, out1, out2);
        printf("Layer02::coroutine1u(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::coroutine1u(): part 2\n");
        co_return in1 + out2 + ret1;
    }
};

class Layer03d : public Layer03
{
public:
    Layer03d(Layer02& layer02)
        : Layer03(layer02)
    {}

    async_task<int> coroutine1d(int in1) override
    {
        printf("Layer03::coroutine1d(): part 1\n");
        int out1 = -1;
        int ret1 = co_await m_layer02.coroutine1d(in1, out1);
        printf("Layer03::coroutine1d(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::coroutine1d(): part 2\n");
        co_return in1 + out1 + ret1;
    }

    // Not used so far
    async_task<int> coroutine2d(int in1)
    {
        printf("Layer03::coroutine2d(): part 1\n");
        int out1 = -1;
        int ret1 = co_await m_layer02.coroutine1d(in1, out1);
        printf("Layer03::coroutine2d(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::coroutine2d(): part 2\n");
        co_return in1 + out1 + ret1;
    }

    /**
     * @brief upstream coroutine at the highest level: we invoke our own downstream coroutine
     * that will go down the protocol stack
     * Note: At the co_await statement:
     *     Warning C26811: Lifetime of the memory referenced by parameter 'out1' might end by the time the coroutine is resumed.
     * https://learn.microsoft.com/en-us/cpp/code-quality/c26811?view=msvc-170
     */
    async_task<int> coroutine1u(int in1, int& out1, int& out2) override
    {
        printf("Layer03::coroutine1u(): part 1\n");
        int ret1 = co_await coroutine1d(in1);
        printf("Layer03::coroutine1u(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer03::coroutine1u(): part 2\n");
        co_return in1 + ret1;
    }
};

/**
 * Object declaration section
 *
 */
 // Forward declaration of objects
extern Layer01d layer01;
extern Layer02d layer02;
extern Layer03d layer03;

//                            lower layer    upper layer
RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{ remoteObj1,    layer01 };
Layer01d layer01            { remoteObj1co,  layer02 };
Layer02d layer02            { layer01,       layer03 };
Layer03d layer03            { layer02                };

EventQueue eventQueue;

/**
 * @brief main function
 *
 */
int main()
{
    printf("main();\n");
    // Downstream example
    async_task<int> t1 = layer03.coroutine1d(2);
    // Upstream example
    async_task<int> t2 = remoteObj1co.start_op1u(3, 4);
    eventQueue.run();
    int ret1 = t1.get_result();
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = t2.get_result();
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
