/**
 * @file p1010-async-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

class RemoteObject1 {
public:
    void sendc_op1(int in11, int in12, lambda_3int_t lambda) {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in11, in12);
        eventQueue.push([lambda]() { lambda(1, 2, 3); });
    }
};

RemoteObject1 remoteObj1;

struct Class01 {
    void function1() {
        printf("Class03::function1(): part 1\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { 
                this->function1_cb(out1, out2, ret1); 
            });
    }

    void function1_cb(int out11, int out12, int ret1) {
        printf("Class03::function1_cb(out11 = %d, out12 = %d, ret1 = %d)\n", out11, out12, ret1);
        printf("Class03::function1_cb(): part 2\n");
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    connect(event1, []() { class01.function1(); });
    connect(event2, []() { class01.function1(); });
    eventQueue.run();
    return 0;
}
