/**
 * @file p1050co.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1050CO_H_
#define _P1050CO_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commservice.h>

#include "p1050.h"

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
        printf("RemoteObjectImplCo::start_write(p, bytestowrite = %d)\n", bytestowrite);
        start_write_impl(index, p, bytestowrite);
        return { this, index };
    }

    async_operation<bool> start_read(char* p, int length)
    {
        int index = get_free_index();
        printf("RemoteObjectImplCo::start_read(p, length = %d)\n", length);
        start_read_impl(index, p, length);
        return { this, index };
    }

protected:
    void start_write_impl(int idx, char* p, int bytestowrite);
    void start_read_impl(int idx, char* p,  int length);

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

    // User API
    /**
     * @brief op1 has the same parameter list as op1 in class RemoteObject1
     * but returns async_task<int> instead of int.
     * Note that this coroutine uses reference paraneters (out1 and out2).
     * This is not recommended because of possible lifetime issues.
     * See the next variant with only in paraneters.
     * @param in1
     * @param in2
     * @param out2
     * @param out2
     * @return async_task<int>
     */
    async_task<int> op1(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1Co::op1(%d, %d, %d, %d)\n", in1, in2, out1, out2);

        // Write part
        Buffer writebuffer;
        // Marshall in1 and in2 into the buffer
        // (code not present)
        // Write the buffer (onto the remote object).
        int buflength = writebuffer.length();
        co_await m_remoteObjImplCo.start_write(writebuffer.buffer(), buflength);
        // At this point the write has completed
        
        // Read part
        Buffer readbuffer;
        // Read the buffer
        int buflength2 = readbuffer.length();
        co_await m_remoteObjImplCo.start_read(readbuffer.buffer(), buflength2);
        // Unmarshall out1, out2 and ret from readbuffer
        // (code not present)
        // At this point the write has completed
        // Because we do not have a communication framework,
        // we simulate reading by filling in the output values manually.
        out1 = 1;
        out2 = 2;
        int ret = in1 + in2;

        // return to the caller
        co_return ret;
    }

    async_task<op1_ret_t> op1(int in1, int in2)
    {
        printf("RemoteObject1Co::op1(%d, %d)\n", in1, in2);

        // Write part
        Buffer writebuffer;
        // Marshall in1 and in2 into the buffer
        // (code not present)
        // Write the buffer (onto the remote object).
        co_await m_remoteObjImplCo.start_write(writebuffer.buffer(), writebuffer.length());
        // At this point the write has completed

        // Read part
        Buffer readbuffer;
        // Read the buffer
        co_await m_remoteObjImplCo.start_read(readbuffer.buffer(), readbuffer.length());
        // Unmarshall out1, out2 and ret from readbuffer
        // (code not present)
        // At this point the write has completed
        // Because we do not have a communication framework,
        // we simulate reading by filling in the return value manually.
        op1_ret_t ret;
        ret.out1 = 1;
        ret.out2 = 2;
        ret.ret = in1 + in2;

        // return to the caller
        co_return ret;
    }
 
private:
    RemoteObjectImplCo& m_remoteObjImplCo;
};

#endif
