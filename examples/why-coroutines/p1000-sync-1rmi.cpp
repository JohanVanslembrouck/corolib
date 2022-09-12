/**
 * @file p1000-sync-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

// -------------------------------------------------

class RemoteObject1 {
public:
    int op1(int in11, int in12, int& out11, int& out12) {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in11, in12, out11, out12);
        // Just sleep a while to simulate waiting for the response
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out11 = 1;
        out12 = 2;
        return 3; 
    }
};

RemoteObject1 remoteObj1;

// -------------------------------------------------

struct Class01 {
    int function1() {
        printf("Class01::function1(): part 1\n");
        ret1 = remoteObj1.op1(in11, in12, out11, out12);
        printf("Class01::function1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Class01::function1(): part 2\n");
        return ret1;
    }
};

Class01 class01;

// -------------------------------------------------

int main() {
    printf("main();\n");
    connect(event1, []() { class01.function1(); });
    //connect(event2, []() { class01.function1(); });
    eventQueue.run();
    return 0;
}
