/**
 * @file p1360-async-nested-loop.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1350.h"                              // difference with p1310-async-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1310-async-nested-loop.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1310-async-nested-loop.cpp

class Class01
{
public:
    void function1()
    {
        printf("Class01::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a()
    {
        printf("Class01::function1a(): counter = %d\n", counter);
        if (j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < MAX_MSG_LENGTH) {
                msg = Msg(i * 10);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
    }
    
private:
    int i = 0, j = 0;
    Msg msg;
    int counter = 0;
};

Class01 class01a;
Class01 class01b;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01a.function1();
    class01b.function1();
    eventQueue.run();
    return 0;
}
