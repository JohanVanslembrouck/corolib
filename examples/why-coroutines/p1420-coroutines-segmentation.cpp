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

class RemoteObjectImpl : public CommService
{
public:
    async_operation<void> start_write_segment(char* p, int offset)
    {
        int index = get_free_index();
        //printf("RemoteObjectImpl::start_write_segment(p, offset = %d): index = %d\n", offset, index);
        printf("RemoteObjectImpl::start_write_segment(p, offset = %d)\n", offset);
        async_operation<void> ret{ this, index };
        start_write_segment_impl(index, p, offset);
        return ret;
    }

    async_operation<bool> start_read_segment(char* p, int offset, int segment_length)
    {
        int index = get_free_index();
        //printf("RemoteObjectImpl::start_read_segment(p, offset = %d, segment_length = %d): index = %d\n", offset, segment_length, index);
        printf("RemoteObjectImpl::start_read_segment(p, offset = %d, segment_length = %d)\n", offset, segment_length);
        async_operation<bool> ret{ this, index };
        start_read_segment_impl(index, p, offset, segment_length);
        return ret;
    }

    // Lower level functions
    void sendc_write_segment(char* p, int offset, lambda_void_t lambda)
    {
        eventQueue.push(lambda);
    }

    void sendc_read_segment(char* p, int offset, int segment_length, lambda_void_t lambda)
    {
        eventQueue.push(lambda);
    }

protected:
    void start_write_segment_impl(int idx, char* p, int offset);
    void start_read_segment_impl(int idx, char* p, int offset, int segment_length);
};

void RemoteObjectImpl::start_write_segment_impl(int idx, char* p, int offset)
{
    sendc_write_segment(p, offset,
        [this, idx]()
        {
            print(PRI2, "%p: RemoteObjectImpl::start_ws(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImpl::start_ws(%d): om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImpl::start_ws(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

void RemoteObjectImpl::start_read_segment_impl(int idx, char* p, int offset, int segment_length)
{
    sendc_read_segment(p, offset, segment_length,
        [this, offset, segment_length, idx]()
        {
            print(PRI2, "%p: RemoteObjectImpl::start_rs(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<bool>* om_async_operation_t =
                dynamic_cast<async_operation<bool>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImpl::start_rs(%d): om_async_operation_t->set_result()\n", this, idx);
                bool ret = (offset > segment_length);
                om_async_operation_t->set_result(ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImpl::start_rs(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

RemoteObjectImpl remoteObjImpl;

struct RemoteObject1
{
    async_task<Msg> op1(Msg msg)
    {
        Buffer buf;
        Msg res;
        printf("RemoteObject1::op1()\n");
        // Marshall msg into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject1::op1(): calling write_segment: offset = %d\n", offset);
            co_await remoteObjImpl.start_write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject1::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await remoteObjImpl.start_read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall Msg from buf2
        co_return res;
    }
};

RemoteObject1 remoteObject1;

struct Class01
{
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::coroutine1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class01::coroutine1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class01::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<Msg> op1 = remoteObject1.op1(msg);
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
    //connect(event2, []() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
