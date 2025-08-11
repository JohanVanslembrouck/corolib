/**
 * @file p1010-async-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <memory>

#include "common.h"
#include "eventqueue.h"
#include "p1000.h"

/**
 * @brief Asynchronous version of Class01 in p1000-sync-1rmi.cpp
 *
 */
class Class01
{
private:
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("function1_ctxt_t::~function1_ctxt_t()\n"); }

        int in1;
        int in2;
        int* ret;
    };

public:
    /**
     * @brief First variant.
     * It uses a send_op1 function that takes a void* as first parameter
     *
     */
    void function1(int in1, int in2, int& ret)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1, in2, &ret };
        printf("Class01::function1(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(ctxt, in1, in2,
            [this](void* context, int out1, int out2, int ret1)
            {
                this->function1_cb(context, out1, out2, ret1);
            });
    }

protected:
    /**
     * @brief callback function with the out parameters and the return value
     *
     */
    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Class01::function1_cb(context = %p, out1 = %d, out2 = %d, ret1 = %d)\n", context, out1, out2, ret1);
        function1_ctxt_t* ctxt = static_cast<function1_ctxt_t*>(context);
        printf("Class01::function1_cb(): ctxt->in1 = %d, ctxt->in2 = %d, ctxt->ret = %p, ctxt->ret = %d)\n", 
            ctxt->in1, ctxt->in2, ctxt->ret, *ctxt->ret);
        *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
        delete ctxt;
    }

public:
    /**
     * @brief Second variant.
     * The context is now passed via the lambda capture, not as the first parameter of sendc_op1.
     *
     */
    void function1a(int in1, int in2, int& ret)
    {
        printf("Class01::function1a(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1, in2, &ret };
        printf("Class01::function1a(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(in1, in2,
            [this, ctxt](int out1, int out2, int ret1)
            {
                this->function1a_cb(ctxt, out1, out2, ret1);
            });
    }

protected:
    /**
     * @brief callback function with the out parameters and the return value.
     * In contrast to function1_cb (see above), this variant is passed the correct context type: no cast required.
     */
    void function1a_cb(function1_ctxt_t* ctxt, int out1, int out2, int ret1)
    {
        printf("Class01::function1a_cb(ctxt = %p, out1 = %d, out2 = %d, ret1 = %d)\n", ctxt, out1, out2, ret1);
        printf("Class01::function1a_cb(): ctxt->in1 = %d, ctxt->in2 = %d, ctxt->ret = %p, ctxt->ret = %d)\n",
            ctxt->in1, ctxt->in2, ctxt->ret, *ctxt->ret);
        *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
        delete ctxt;
    }
    
public:
    /**
     * @brief alternative version of function1 that avoids the introduction
     * of a callback function by placing part 2 of the original synchronous function
     * in the body of the lambda.
     *
     */
    void function1alt1(int in1, int in2, int& ret)
    {
        printf("Class01::function1alt1(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1, in2, &ret };
        printf("Class01::function1alt1(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(ctxt, in1, in2,
            [](void* context, int out1, int out2, int ret1)
            {
                function1_ctxt_t* cntx = static_cast<function1_ctxt_t*>(context);
                printf("Class01::function1alt1::handler(context = %p, out1 = %d, out2 = %d, ret1 = %d)\n", context, out1, out2, ret1);
                printf("Class01::function1alt1::handler(): cntx->in1 = %d, cntx->in2 = %d, cntx->ret = %p)\n", cntx->in1, cntx->in2, cntx->ret);
                *cntx->ret = cntx->in1 + cntx->in2 + out1 + out2 + ret1;
                delete cntx;
            });
    }

    void function1alt2(int in1, int in2, int& ret)
    {
        printf("Class01::function1alt2(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        function1_ctxt_t* ctxt = new function1_ctxt_t{ in1, in2, &ret };
        printf("Class01::function1alt2(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(in1, in2,
            [ctxt](int out1, int out2, int ret1)
            {
                printf("Class01::function1alt2::handler(ctxt = %p, out1 = %d, out2 = %d, ret1 = %d)\n", ctxt, out1, out2, ret1);
                printf("Class01::function1alt2::handler(): cntx->in1 = %d, cntx->in2 = %d, cntx->ret = %p)\n", ctxt->in1, ctxt->in2, ctxt->ret);
                *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
                delete ctxt;
            });
    }

    void function1alt3(int in1, int in2, int& ret)
    {
        printf("Class01::function1alt3(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        std::shared_ptr<function1_ctxt_t> ctxt = std::make_shared<function1_ctxt_t>(in1, in2, &ret);

        remoteObj1.sendc_op1(in1, in2,
            [ctxt](int out1, int out2, int ret1)
            {
                printf("Class01::function1alt3::handler(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
                printf("Class01::function1alt3::handler(): cntx->in1 = %d, cntx->in2 = %d, cntx->ret = %p)\n", ctxt->in1, ctxt->in2, ctxt->ret);
                *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
            });
    }

private:
    RemoteObject1 remoteObj1;
};

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    Class01 class01;

#if 0
    int ret1 = -1;
    class01.function1(11, 12, ret1);
    eventQueue.run();

    int ret2 = -1;
    class01.function1a(11, 12, ret2);
    eventQueue.run();
    
    int ret3 = -1;
    class01.function1alt1(11, 12, ret3);
    eventQueue.run();
    
    int ret4 = -1;
    class01.function1alt2(11, 12, ret4);
    eventQueue.run();
    
    int ret5 = -1;
    class01.function1alt3(11, 12, ret5);
    eventQueue.run();

    printf("\n");
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    printf("main(): ret5 = %d\n", ret5);
#else
    int ret1a = -1;
    class01.function1(11, 12, ret1a);
    int ret1b = -1;
    class01.function1(21, 22, ret1b);
    int ret1c = -1;
    class01.function1(31, 32, ret1c);
    int ret1d = -1;
    class01.function1(41, 42, ret1d);

    int ret2a = -1;
    class01.function1a(11, 12, ret2a);
    int ret2b = -1;
    class01.function1a(21, 22, ret2b);
    int ret2c = -1;
    class01.function1a(31, 32, ret2c);
    int ret2d = -1;
    class01.function1a(41, 42, ret2d);

    int ret3a = -1;
    class01.function1alt1(11, 12, ret3a);
    int ret3b = -1;
    class01.function1alt1(21, 22, ret3b);
    int ret3c = -1;
    class01.function1alt1(31, 32, ret3c);
    int ret3d = -1;
    class01.function1alt1(41, 42, ret3d);

    int ret4a = -1;
    class01.function1alt2(11, 12, ret4a);
    int ret4b = -1;
    class01.function1alt2(21, 22, ret4b);
    int ret4c = -1;
    class01.function1alt2(31, 32, ret4c);
    int ret4d = -1;
    class01.function1alt2(41, 42, ret4d);

    int ret5a = -1;
    class01.function1alt3(11, 12, ret5a);
    int ret5b = -1;
    class01.function1alt3(21, 22, ret5b);
    int ret5c = -1;
    class01.function1alt3(31, 32, ret5c);
    int ret5d = -1;
    class01.function1alt3(41, 42, ret5d);

    eventQueue.run();
    
    printf("\n");
    printf("main(): ret1a = %d\n", ret1a);
    printf("main(): ret1b = %d\n", ret1b);
    printf("main(): ret1c = %d\n", ret1c);
    printf("main(): ret1d = %d\n", ret1d);

    printf("main(): ret2a = %d\n", ret2a);
    printf("main(): ret2b = %d\n", ret2b);
    printf("main(): ret2c = %d\n", ret2c);
    printf("main(): ret2d = %d\n", ret2d);

    printf("main(): ret3a = %d\n", ret3a);
    printf("main(): ret3b = %d\n", ret3b);
    printf("main(): ret3c = %d\n", ret3c);
    printf("main(): ret3d = %d\n", ret3d);

    printf("main(): ret4a = %d\n", ret4a);
    printf("main(): ret4b = %d\n", ret4b);
    printf("main(): ret4c = %d\n", ret4c);
    printf("main(): ret4d = %d\n", ret4d);

    printf("main(): ret5a = %d\n", ret5a);
    printf("main(): ret5b = %d\n", ret5b);
    printf("main(): ret5c = %d\n", ret5c);
    printf("main(): ret5d = %d\n", ret5d);
#endif
    return 0;
}
