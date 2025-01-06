/**
 * @file p1350co.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1350CO_H_
#define _P1350CO_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

#include "p1350.h"

using namespace corolib;

class RemoteObjectImplCo : public CommService
{
public:
    RemoteObjectImplCo(RemoteObjectImpl& remoteObjectImpl)
        : m_remoteObjectImpl(remoteObjectImpl)
    {
    }

    async_operation<void> start_write(char* p, int bytestowrite)
    {
        int index = get_free_index();
        //printf("RemoteObjectImplCo::start_write(p, bytestowrite = %d): index = %d\n", bytestowrite, index);
        printf("RemoteObjectImplCo::start_write(p, bytestowrite = %d)\n", bytestowrite);
        start_write_impl(index, p, bytestowrite);
        return { this, index };
    }

    async_operation<bool> start_read(char* p, int length)
    {
        int index = get_free_index();
        //printf("RemoteObjectImplCo::start_read(p, length = %d): index = %d\n", length, index);
        printf("RemoteObjectImplCo::start_read(p, length = %d)\n", length);
        start_read_impl(index, p, length);
        return { this, index };
    }

protected:
    void start_write_impl(int idx, char* p, int bytestowrite);
    void start_read_impl(int idx, char* p, int length);

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

    async_task<int> op1(Msg& msg)
    {
        // Write part
        Buffer writebuffer;
        // Marshall in1 and in2 into the buffer
        // (code not present)
        // Write the buffer (onto the remote object).
        co_await m_remoteObjImplCo.start_write(writebuffer.buffer(), msg.m_length);
        // At this point the write has completed

        // Read part
        Buffer readbuffer;
        // Read the buffer
        co_await m_remoteObjImplCo.start_read(readbuffer.buffer(), 4);
        // At this point the read has completed

        co_return 0;
    }

private:
    RemoteObjectImplCo& m_remoteObjImplCo;
};

#endif
