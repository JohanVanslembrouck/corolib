/**
 * @file p1370-coroutines-nested-loop.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1350co.h"                                        // difference with p1320-coroutines-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                             // difference with p1320-coroutines-nested-loop.cpp
RemoteObjectImplCo remoteObjImplco{ remoteObjImpl };        // difference with p1320-coroutines-nested-loop.cpp
RemoteObject1Co remoteObj1co{ remoteObjImplco };            // difference with p1320-coroutines-nested-loop.cpp

class Class01
{
public:
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::coroutine1()\n");
        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::coroutine1(): i = %d\n", i);
            Msg msg(i * 10);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class02::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<int> op1 = remoteObj1co.op1(msg);    // difference with p1320-coroutines-nested-loop.cpp
                int ret1 = co_await op1;
                (void)ret1;
            }
        }
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        printf("Class01::function1a(): time_taken = %f s\n", time_taken / 1000000000.0);
    }
};

Class01 class01a;
Class01 class01b;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    async_task<void> t1 = class01a.coroutine1();
    async_task<void> t2 = class01b.coroutine1();
    eventQueue.run();
    t1.wait();
    t2.wait();
    return 0;
}
