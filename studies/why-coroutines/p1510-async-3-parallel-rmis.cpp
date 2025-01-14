/**
 * @file p1510-async-3-parallel-rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "eventqueue.h"
#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

class Class01
{
private:
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("function1_ctxt_t::~function1_ctxt_t()\n"); }

        bool callfinished[3]{ false, false, false };
        int result[3]{ 0, 0, 0 };
    };

public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        function1_ctxt_t* ctxt = new function1_ctxt_t;

        remoteObj1.sendc_op1(in1, in2, 
            [this, ctxt](int out1, int out2, int ret1) { this->function1a(ctxt, 0, out1, out2, ret1); });
        remoteObj2.sendc_op1(in1, in2, 
            [this, ctxt](int out1, int out2, int ret1) { this->function1a(ctxt, 1, out1, out2, ret1); });
        remoteObj3.sendc_op1(in1, in2, 
            [this, ctxt](int out1, int out2, int ret1) { this->function1a(ctxt, 2, out1, out2, ret1); });
    }

protected:
    void function1a(function1_ctxt_t* ctxt, int index, int out1, int out2, int ret1)
    {
        printf("Class01::function1a(%d, %d, %d)\n", out1, out2, ret1);
        ctxt->callfinished[index] = true;
        ctxt->result[index] = ret1;
        if (ctxt->callfinished[0] && ctxt->callfinished[1] && ctxt->callfinished[2])
            printf("Class01::function1a: result = %d\n", ctxt->result[0] + ctxt->result[1] + ctxt->result[2]);
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01.function1(11, 12);
    class01.function1(21, 22);
    class01.function1(31, 32);
    class01.function1(41, 42);
    eventQueue.run();
    return 0;
}
