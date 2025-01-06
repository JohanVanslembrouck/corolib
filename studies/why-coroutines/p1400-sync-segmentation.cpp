/**
 * @file p1400-sync-segmentation.cpp
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
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                Msg res = remoteObj1.op1(msg);
                (void)res;
                // Do something with msg
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01a;
Class01 class01b;

int main() {
    printf("main();\n");
    class01a.function1();
    class01b.function1();
    return 0;
}
