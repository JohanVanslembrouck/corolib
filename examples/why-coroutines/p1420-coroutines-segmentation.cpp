/**
 * @file p1420-coroutines-segmentation.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "common.h"
#include "variables.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

#include "p1400co.h"

RemoteObjectImpl remoteObjImpl;
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
                (void)res;
                // Do something with msg
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
