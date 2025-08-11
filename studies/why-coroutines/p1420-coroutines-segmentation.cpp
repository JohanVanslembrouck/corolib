/**
 * @file p1420-coroutines-segmentation.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <chrono>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "p1400co.h"

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
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<Msg> op1 = remoteObj1co.op1(msg);
                Msg res = co_await op1;
                (void)res;
                // Do something with msg
            }
        }
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        printf("Class01::function1a(): time_taken = %f s\n", time_taken / 1000000000.0);
    }

private:
    RemoteObjectImpl remoteObjImpl;
    RemoteObjectImplCo remoteObjImplco{ remoteObjImpl };
    RemoteObject1Co remoteObj1co{ remoteObjImplco };
};

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    Class01 class01a;
    Class01 class01b;
    async_task<void> t1 = class01a.coroutine1();
    async_task<void> t2 = class01b.coroutine1();
    eventQueue.run();
    t1.wait();
    t2.wait();
    return 0;
}
