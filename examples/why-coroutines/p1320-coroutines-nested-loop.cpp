/**
 * @file p1320-coroutines-nested-loop.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1300co.h"

RemoteObject1 remoteObj1;
RemoteObject1Co remoteObj1co{remoteObj1};

class Class01
{
public:
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::coroutine1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::coroutine1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class02::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_operation<int> op1 = remoteObj1co.start_op1(msg);
                int ret1 = co_await op1;
                (void)ret1;
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    async_task<void> t1 = class01.coroutine1();
    //async_task<void> t2 = class01.coroutine1();
    eventQueue.run();
    return 0;
}
