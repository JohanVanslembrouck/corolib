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

#include "p1400.h"

RemoteObjectImpl remoteObjImpl;

using namespace corolib;

class RemoteObjectImplCo : public CommService
{
public:
    RemoteObjectImplCo(RemoteObjectImpl& remoteObjectImpl)
        : m_remoteObjectImpl(remoteObjectImpl)
    {}
    
    void init()
    {
        m_remoteObjectImpl.init();
    }

    async_operation<void> start_write_segment(char* p, int offset, int bytestowrite)
    {
        int index = get_free_index();
        //printf("RemoteObjectImplCo::start_write_segment(p, offset = %d): index = %d\n", offset, index);
        printf("RemoteObjectImplCo::start_write_segment(p, offset = %d)\n", offset);
        start_write_segment_impl(index, p, offset, bytestowrite);
        return { this, index };
    }

    async_operation<bool> start_read_segment(char* p, int offset, int length)
    {
        int index = get_free_index();
        //printf("RemoteObjectImplCo::start_read_segment(p, offset = %d, length = %d): index = %d\n", offset, length, index);
        printf("RemoteObjectImplCo::start_read_segment(p, offset = %d, length = %d)\n", offset, length);
        start_read_segment_impl(index, p, offset, length);
        return { this, index };
    }

protected:
    void start_write_segment_impl(int idx, char* p, int offset, int bytestowrite);
    void start_read_segment_impl(int idx, char* p, int offset, int length);

private:
    RemoteObjectImpl m_remoteObjectImpl;
};

void RemoteObjectImplCo::start_write_segment_impl(int idx, char* p, int offset, int bytestowrite)
{
    m_remoteObjectImpl.sendc_write_segment(p, offset, bytestowrite,
        [this, idx]()
        {
            print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

void RemoteObjectImplCo::start_read_segment_impl(int idx, char* p, int offset, int length)
{
    m_remoteObjectImpl.sendc_read_segment(p, offset, length,
        [this, idx](bool result)
        {
            print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d)\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<bool>* om_async_operation_t =
                dynamic_cast<async_operation<bool>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d): om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->set_result(result);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d): Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

RemoteObjectImplCo remoteObjImplco{remoteObjImpl};

class RemoteObject1Co
{
public:
    async_task<Msg> op1(Msg msg)
    {
        // Write part
        Buffer writebuffer;
        printf("RemoteObject1Co::op1()\n");
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        int buflength = writebuffer.length();
        for (int offset = 0; offset < buflength; offset += SEGMENT_LENGTH)
        {
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            printf("RemoteObject1Co::op1(): calling write_segment: offset = %d\n", offset);
            co_await remoteObjImplco.start_write_segment(writebuffer.buffer(), offset, bytestowrite);
        }
        
        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        remoteObjImplco.init();
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until start_read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1Co::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await remoteObjImplco.start_read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        co_return res;
    }
};

RemoteObject1Co remoteObj1co;

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
                printf("Class01::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<Msg> op1 = remoteObj1co.op1(msg);
                Msg res = co_await op1;
                // Do something with msg
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01.coroutine1(); });
    //eventQueue.push([]() { class01.coroutine1(); });
    eventQueue.run();
    return 0;
}
