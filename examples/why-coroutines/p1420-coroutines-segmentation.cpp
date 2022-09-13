/**
 * @file p1420-coroutines-segmentation.cpp
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

struct op1_ret_t
{
    int out1;
    int out2;
    int ret;
};

struct op2_ret_t
{
    int out1;
    int ret;
};

// -----------------------------------------------------------------------------

class RemoteObjectImpl : public CommService {
public:
    async_operation<void> start_write_segment(char* p, int offset)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObjectImpl::start_write_segment(): index = %d\n", this, index);
        async_operation<void> ret{ this, index };
        start_ws(index);
        return ret;
    }

    async_operation<bool> start_read_segment(char* p, int offset, int segment_length)
    {
        int index = get_free_index();
        print(PRI1, "%p: RemoteObjectImpl::start_read_segment(): index = %d\n", this, index);
        async_operation<bool> ret{ this, index };
        start_rs(index);
        return ret;
    }

    lambda_void_t operation;

protected:
    void start_ws(int idx);
    void start_rs(int idx);
};

void RemoteObjectImpl::start_ws(int idx)
{
    lambda_void_t* operation = new lambda_void_t(
        [this, idx]()
        {
            print(PRI1, "%p: RemoteObjectImpl::start_ws(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObjectImpl::start_ws(%d): om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObjectImpl::start_ws(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });

    eventQueue.push(
        [operation]()
        {
            (*operation)();
            delete operation;
        });
}

void RemoteObjectImpl::start_rs(int idx)
{
    operation = [this, idx]()
    {
        print(PRI1, "%p: RemoteObjectImpl::start_rs(%d)\n", this, idx);

        async_operation_base* om_async_operation = m_async_operations[idx];
        async_operation<bool>* om_async_operation_t =
            dynamic_cast<async_operation<bool>*>(om_async_operation);

        if (om_async_operation_t)
        {
            print(PRI1, "%p: RemoteObjectImpl::start_rs(%d): om_async_operation_t->set_result()\n", this, idx);
            bool ret = true;
            om_async_operation_t->set_result(ret);
            om_async_operation_t->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "%p: RemoteObjectImpl::start_rs(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
        }
    };

    eventQueue.push(operation);
}

RemoteObjectImpl remoteObjImpl;

struct RemoteObject2 {
    async_task<Msg> op1(Msg msg) {
        Buffer buf;
        Msg res;
        printf("RemoteObject2::op1()\n");
        // Marshall msg into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject2::op1(): calling write_segment: offset = %d\n", offset);
            co_await remoteObjImpl.start_write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject2::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await remoteObjImpl.start_read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall Msg from buf2
        co_return res;
    }
};

RemoteObject2 remoteObject2a;

struct Class01
{
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class03::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class03::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<Msg> op1 = remoteObject2a.op1(msg);
                Msg res = co_await op1;
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
