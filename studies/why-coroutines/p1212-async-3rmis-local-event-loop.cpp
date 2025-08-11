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

class Class01
{
public:
    void function1(int in1, int in2, int testval)
    {
        int lret1 = -1;
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2,
            [this, &lret1](int out1, int out2, int ret1) {
                lret1 = this->callback1(out1, out2, ret1);
            });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (lret1 == testval) {
            remoteObj2.sendc_op2(in1, in2, 
                [this](int out1, int ret1) {
                    this->callback2(out1, ret1);
                });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in1, 
                [this](int out1, int out2, int ret1) {
                    this->callback3(out1, out2, ret1);
                });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

protected:
    int callback1(int out1, int out2, int ret1)
    { 
        printf("Class01::callback1(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        // copy to local variables
        return ret1;
    }
    
    void callback2(int out3, int ret2) { 
        printf("Class01::callback2(out3 = %d, ret2 = %d)\n", out3, ret2);
        // copy to local variables
    }
    
    void callback3(int out4, int out5, int ret3) {
        printf("Class01::callback3(out4 = %d, out5 = %d, ret3 = %d)\n", out4, out5, ret3);
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1 remoteObj2;
    RemoteObject1 remoteObj3;
};

EventQueue eventQueue;

int main() {
    printf("main2();\n");
    Class01 class01;
    class01.function1(11, 12, 10);
    printf("\n");
    class01.function1(11, 12, 23);
    eventQueue.run();
    return 0;
}
