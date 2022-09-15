/**
 * @file p1320-coroutines-nested-loop.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

class RemoteObj1 : public CommService
{
public:  
    async_operation<int> start_op1(Msg msg)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObj1::start_op1(): index = %d\n", this, index);
        async_operation<int> ret{ this, index };
        start_op1_impl(index, msg);
        return ret;
    }

    // Lower level function
    void sendc_op1(Msg& msg, lambda_void_t lambda)
    {
        printf("RemoteObject1::sendc_op1(msg, lambda)\n");
        eventQueue.push(lambda);
    }

protected:
    void start_op1_impl(const int idx, Msg& msg);
};

void RemoteObj1::start_op1_impl(const int idx, Msg& msg)
{
    print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

    sendc_op1(msg, 
        [this, idx]()
        {
            print(PRI1, "%p: RemoteObj1::start_op1_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<int>* om_async_operation_t =
                dynamic_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->set_result(1);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObj1::start_op1_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

RemoteObj1 remoteObj1;

struct Class01
{
    async_task<void> coroutine1() {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class02::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_operation<int> op1 = remoteObj1.start_op1(msg);
                int i = co_await op1;
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01;

int main() {
    printf("main();\n");
    connect(event1, []() { class01.coroutine1(); });
    connect(event2, []() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}