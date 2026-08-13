/**
 * @file p1400-sync-segmentation.cpp
 * @brief Example with 1 synchronous RMI called in a nested loop (a loop in a loop).
 * This program is not reactive (responsive) because the RMI blocks the program
 * for the duration of the RMI.
 * 
 * Notice how this implementation is very similar to that in p1300-sync-nested-loop.cpp.
 * Differences have been marked in the code.
 * 
 * @author Johan Vanslembrouck
 */

#include <stdio.h>
#include <chrono>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1400.h"          // difference with p1300-sync-nested-loop.cpp

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
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);                                     // difference with p1300-sync-nested-loop.cpp
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                Msg res = remoteObj1.op1(msg);              // difference with p1300-sync-nested-loop.cpp
                (void)res;                                  // difference with p1300-sync-nested-loop.cpp
                // Do something with res
            }
        }
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        printf("Class01::function1a(): time_taken = %f s\n", time_taken / 1000000000.0);
    }

private:
    RemoteObjectImpl remoteObjImpl;                         // difference with p1300-sync-nested-loop.cpp
    RemoteObject1 remoteObj1{ remoteObjImpl };              // difference with p1300-sync-nested-loop.cpp
};

int main() {
    printf("main(): begin\n");
    Class01 class01a;
    Class01 class01b;
    class01a.function1();
    class01b.function1();
    printf("main(): end\n");
    return 0;
}
