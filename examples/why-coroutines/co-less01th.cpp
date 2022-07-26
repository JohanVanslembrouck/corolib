/**
 * @file co-less01th.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <thread>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

// -----------------------------------------------------------------------------

class RemoteObject1 {
public:
    int op1(int in11, int in12, int& out11, int& out12) {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in11, in12, out11, out12);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0; 
    }

    int op2(int in21, int in22, int& out21) {
        printf("RemoteObject1::op2(%d, %d, %d)\n", in21, in22, out21);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
  
    int op3(int in31, int& out31, int& out32) {
        printf("RemoteObject1::op3(%d, %d, %d)\n", in31, out31, out32);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
};

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

// -----------------------------------------------------------------------------

struct Class01 {
    void function1() {
        printf("Class01::function1()\n");
        ret1 = remoteObj1.op1(in11, in12, out11, out12);
        // 1 Do stuff
        if (ret1 == val1) {
            ret2 = remoteObj2.op2(in21, in22, out21);
            // 2 Do stuff
        }
        else {
            ret3 = remoteObj3.op3(in31, out31, out32);
            // 3 Do stuff
        }
    }
    void function2() { }
};

Class01 class01;

int main() {
    printf("main();\n");
    connect(event1, []() { std::thread th(&Class01::function1, &class01); th.join(); });
    connect(event2, []() { std::thread th(&Class01::function1, &class01); th.join(); });
    eventQueue.run();
    return 0;
}
