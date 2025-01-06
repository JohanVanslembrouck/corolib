/**
 * @file p1060-async-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "eventqueue.h"
#include "p1050.h"                              // difference with p1010-async-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1010-async-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1010-async-1rmi.cpp

/**
 * @brief Asynchronous version of Class01 in p1050-sync-1rmi.cpp
 *
 */
class Class01
{
public:

    struct function1_cxt_t
    {
        int in1;
        int in2;
        int* ret;
    };

    /**
     * @brief
     *
     */
    void function1(int in1, int in2, int& ret)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        void* ctxt = new function1_cxt_t{ in1, in2, &ret };
        printf("Class01::function1(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(ctxt,
            [this](void* context, int out1, int out2, int ret1)
            {
                this->function1_cb(context, out1, out2, ret1);
            },
            in1, in2);
    }

    /**
     * @brief callback function with the out parameters and the return value
     *
     */
    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Class01::function1_cb(context = %p, out1 = %d, out2 = %d, ret1 = %d)\n", context, out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);

        printf("Class01::function1_cb(): ctxt->in1 = %d, ctxt->in2 = %d, ctxt->ret = %p, ctxt->ret = %d)\n", 
            ctxt->in1, ctxt->in2, ctxt->ret, *ctxt->ret);
        *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
        delete ctxt;
    }

    /**
     * @brief alternative version of function1 that avoids the introduction
     * of a callback function by placing part 2 of the original synchronous function
     * in the body of the lambda.
     *
     */
    void function1alt(int in1, int in2, int& ret)
    {
        printf("Class01::function1alt(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        void* ctxt = new function1_cxt_t{ in1, in2, &ret };
        printf("Class01::function1alt(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(ctxt,
            [this](void* context, int out1, int out2, int ret1)
            {
                function1_cxt_t* cntx = static_cast<function1_cxt_t*>(context);
                printf("Class01::function1alt::handler(context = %p, out1 = %d, out2 = %d, ret1 = %d)\n", context, out1, out2, ret1);
                printf("Class01::function1alt::handler(): cntx->in1 = %d, cntx->in2 = %d, cntx->ret = %p)\n", cntx->in1, cntx->in2, cntx->ret);
                *cntx->ret = cntx->in1 + cntx->in2 + out1 + out2 + ret1;
                delete cntx;
            },
            in1, in2);
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    int ret1 = -1;
    class01.function1(11, 12, ret1);
    int ret2 = -1;
    class01.function1alt(21, 22, ret2);
    eventQueue.run();
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
