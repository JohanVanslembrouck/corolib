/**
 *  Filename: co-less02.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

class RemoteObject1 {
public:
    void sendc_op1(int in11, int in12, lambda1 l ) {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in11, in12);
        eventQueue.push([l]() { l(1, 2, 3); });
    }
    void sendc_op2(int in11, int in12, lambda2 l) {
        printf("RemoteObject1::sendc_op2(%d, %d, l)\n", in11, in12);
        eventQueue.push([l]() { l(1, 2); });
    }
    void sendc_op3(int in11, lambda1 l) {
        printf("RemoteObject1::op3(%d, l)\n", in11);
        eventQueue.push([l]() { l(1, 2, 3); });
    }
};

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

// -----------------------------------------------------------------------------

struct Class02 {
    void function1() {
        printf("ClassA2::function1()\n");
        remoteObj1.sendc_op1(in11, in12,
            [this](int out1, int out2, int ret1) { this->callback1(out1, out2, ret1); });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            remoteObj2.sendc_op2(in21, in22, 
                [this](int out1, int ret1) { this->callback2(out1, ret1); });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in31, 
                [this](int out1, int out2, int ret1) { this->callback3(out1, out2, ret1); });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

    void callback1(int out11, int out12, int ret1) { 
        printf("ClassA2::callback1(%d, %d, %d)\n", out11, out12, ret1);
        // copy to local variables 
    }
    void callback2(int out21, int ret2) { 
        printf("ClassA2::callback2(%d, %d)\n", out21, ret2);
        // copy to local variables
    }
    void callback3(int out31, int out32, int ret3) {
        printf("ClassA2::callback3(%d, %d, %d)\n", out31, out32, ret3);
        // copy to local variables
    }
    void function2() { 
        printf("ClassA2::function2()\n");
    }
};

Class02 class02;

int main() {
    printf("main2();\n");
    connect(event1, []() { class02.function1(); });
    connect(event2, []() { class02.function1(); });
    eventQueue.run();
    return 0;
}
