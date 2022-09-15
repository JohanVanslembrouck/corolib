/**
 * @file p1300-sync-nested-loop.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

class RemoteObject1 {
public:
    int op1(Msg& msg) {
        printf("RemoteObject1::op1(msg)\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
};

RemoteObject1 remoteObj1;

struct Class01 {
    void function1() {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class04::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class04::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                int ret1 = remoteObj1.op1(msg);
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
    void function2() { 
        printf("Class01::function2()\n");
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
