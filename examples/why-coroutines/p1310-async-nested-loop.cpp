/**
 * @file p1310-async-nested-loop.cpp
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
    void sendc_op1(Msg& msg, lambda_void_t lambda) {
        printf("RemoteObject1::sendc_op1(msg, lambda)\n");
        eventQueue.push(lambda);
    }
};

RemoteObject1 remoteObj1;

// -----------------------------------------------------------------------------

struct Class01 {
    int i = 0, j = 0;
    Msg msg;
    int counter = 0;

    void function1() {
        printf("Class01::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a() {
        printf("Class01::function1a(): counter = %d\n", counter);
        if (j < nr_msgs_to_send) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < max_msg_length) {
                msg = Msg(i);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
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
