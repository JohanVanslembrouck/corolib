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
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(0, out1, out2, ret1); });
        remoteObj2.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(1, out1, out2, ret1); });
        remoteObj3.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(2, out1, out2, ret1); });
    }

protected:
    void function1a(int index, int out1, int out2, int ret1)
    {
        printf("Class01::function1a(%d, %d, %d)\n", out1, out2, ret1);
        callfinished[index] = true;
        result[index] = ret1;
        if (callfinished[0] && callfinished[1] && callfinished[2])
            printf("Class01::function1a: result = %d\n", result[0] + result[1] + result[2]);
    }
    
private:
    bool callfinished[3]{ false, false, false };
    int result[3]{ 0, 0, 0 };
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01.function1(11, 12);
    class01.function1(21, 22);
    eventQueue.run();
    return 0;
}
