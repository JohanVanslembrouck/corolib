/**
 * @file p1400co.h
 * @brief 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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

protected:
    RemoteObjectImpl& m_remoteObjectImpl;
};

#endif
