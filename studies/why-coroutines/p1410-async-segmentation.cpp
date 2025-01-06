/**
 * @file p1410-async-segmentation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1400.h"

RemoteObjectImpl remoteObjImpl;
RemoteObject1 remoteObj1{remoteObjImpl};

class Class01
{
public:
    void function1()
    {
		counter = 0;
        printf("Class01::function1(): counter = %d\n", counter);
        i = j = 0;
        
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
    }

    void function1a(Msg msgout)
    {
        // Do something with msgout
        printf("Class01::function1a(Msg): counter = %d\n", counter);
        if (j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
            j++;
            counter++;
        }
        else {
            // End of inner loop
            j = 0;
            i++;
            if (i < MAX_MSG_LENGTH) {
                msg = Msg(i);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
                j++;
                counter++;
            }
            else {
                // End of inner and outer loop
                elapsed_time = get_current_time() - start_time;
            }
        }
    }
    
private:
    int i, j;
    Msg msg;
    int counter;
};

Class01 class01a;
Class01 class01b;
Msg gmsg1;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01a.function1();
    class01b.function1();
    eventQueue.run();
    return 0;
}
