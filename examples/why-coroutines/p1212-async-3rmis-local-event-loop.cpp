/**
 * @file p1212-async-3rmis-local-event-loop.cpp
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

int gval1 = 0;
int gret1 = 0;

class Class01
{
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        remoteObj1.sendc_op1(in1, in2,
            [this](int out1, int out2, int ret1) { this->callback1(out1, out2, ret1); });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (gret1 == gval1) {
            remoteObj2.sendc_op2(in1, in2, 
                [this](int out1, int ret1) { this->callback2(out1, ret1); });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in1, 
                [this](int out1, int out2, int ret1) { this->callback3(out1, out2, ret1); });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

    void callback1(int out11, int out12, int ret1)
    { 
        printf("Class01::callback1(out11 = %d, out12 = %d, ret1 = %d)\n", out11, out12, ret1);
        // copy to local variables
        gret1 = ret1;
    }
    
    void callback2(int out21, int ret2) { 
        printf("Class01::callback2(out21 = %d, ret2 = %d)\n", out21, ret2);
        // copy to local variables
    }
    
    void callback3(int out31, int out32, int ret3) {
        printf("Class01::callback3(out31 = %d, out32 = %d, ret3 = %d)\n", out31, out32, ret3);
        // copy to local variables
    }
    
    void function2() { 
        printf("Class01::function2()\n");
    }
};

Class01 class01;

EventQueue eventQueue;

int main() {
    printf("main2();\n");
    class01.function1(11, 12);
    class01.function1(21, 22);
    eventQueue.run();
    return 0;
}
