/**
 *  Filename: co-less03.cpp
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
    void sendc_op1(int in11, int in12, lambda1 l) {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in11, in12);
        eventQueue.push([l]() { l(1, 2, 3); });
    }

    void sendc_op2(int in11, int in12, lambda2 l) {
        printf("RemoteObject1::sendc_op2(%d, %d, l)\n", in11, in12);
        eventQueue.push([l]() { l(1, 2); });
    }

    void sendc_op3(int in11, lambda1 l) {
        printf("RemoteObject1::sendc_op3(%d, l)\n", in11);
        eventQueue.push([l]() { l(1, 2, 3); });
    }
};

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

// -----------------------------------------------------------------------------

struct Class03 {
    void function1() {
        printf("Class03::function1()\n");
        remoteObj1.sendc_op1(in11, in12, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int out11, int out12, int ret1) {
        printf("Class03::function1a(%d, %d, %d)\n", out11, out12, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            remoteObj2.sendc_op2(in21, in22,
                [this](int out1, int ret1){ this->function1b(out1, ret1); });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in31, 
                [this](int out1, int out2, int ret1) { this->function1c(out1, out2, ret1); });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out21, int ret2) {
        printf("Class03::function1b()\n");
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out31, int out32, int ret3) {
        printf("Class03::function1c()\n");
        // 3b Do stuff that needs the result of the RMI
    }

    void function2() {
        printf("Class03::function2()\n");
    }
};

Class03 class03;

int main() {
    printf("main();\n");
    connect(event1, []() { class03.function1(); });
    connect(event2, []() { class03.function1(); });
    eventQueue.run();
    return 0;
}
