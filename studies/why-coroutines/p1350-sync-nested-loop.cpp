/**
 * @file p1350-sync-nested-loop.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <chrono>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1350.h"                              // difference with p1300-sync-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1300-sync-nested-loop.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1300-sync-nested-loop.cpp

class Class01
{
public:
    void function1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class04::function1(): i = %d\n", i);
            Msg msg(i * 10);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class04::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                int ret1 = remoteObj1.op1(msg);
                (void)ret1;
            }
        }
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        printf("Class01::function1a(): time_taken = %f s\n", time_taken / 1000000000.0);
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    class01.function1();
    class01.function1();
    return 0;
}
