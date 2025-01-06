/**
 * @file p1400co.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1400CO_H_
#define _P1400CO_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

#include "p1400.h"

using namespace corolib;

class RemoteObjectImplCo : public CommService
{
public:
    RemoteObjectImplCo(RemoteObjectImpl& remoteObjectImpl)
        : m_remoteObjectImpl(remoteObjectImpl)
    {}
    
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

protected:
    RemoteObjectImpl& m_remoteObjectImpl;
};


class RemoteObject1Co
{
public:
    RemoteObject1Co(RemoteObjectImplCo& remoteObjImplCo)
        : m_remoteObjImplCo(remoteObjImplCo)
    {
    }

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
            co_await m_remoteObjImplCo.start_write_segment(writebuffer.buffer(), offset, bytestowrite);
        }

        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until start_read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1Co::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await m_remoteObjImplCo.start_read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        co_return res;
    }

protected:
    RemoteObjectImplCo& m_remoteObjImplCo;
};

#endif
