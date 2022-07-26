/**
 * @file co-less05.cpp
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
    void sendc_op1(Msg& msg, lambda3 l) {
        printf("RemoteObject1::sendc_op1(msg, l)\n");
        eventQueue.push(l);
    }
};

RemoteObject1 remoteObj1;

// -----------------------------------------------------------------------------

struct Class05 {
    int i, j = 0;
    Msg msg;
    int counter = 0;

    void function1() {
        printf("Class05::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a() {
        printf("Class05::function1a(): counter = %d\n", counter);
        if (j < nr_msgs_to_send) {
            printf("Class05::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < max_msg_length) {
                msg = Msg(i);
                printf("Class05::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
    }

    void function2() {
        printf("Class05::function2()\n");
    }
};

Class05 class05;

int main() {
    printf("main();\n");
    connect(event1, []() { class05.function1(); });
  //connect(event2, []() { class05.function1(); });
    eventQueue.run();
    return 0;
}
