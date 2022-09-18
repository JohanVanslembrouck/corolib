/**
 * @file p1400.h
 * @brief Contains a RemoteObjectImpl(ementation) class that is common to all p1400-* variants.
 * RemoteObjectImpl contains functions to write and read a segment (synchronous version)
 * or to start writing and reading a segment (asynchronous version).
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1400_H_
#define _P1400_H_

#include <assert.h>

#include "common.h"
#include "variables.h"

class RemoteObjectImpl
{
public:
    void init()
    {
        m_readOffset = 0;
    }
    
    /**
     * @brief synchronous version
     * @param p: pointer to the buffer
     * @param offset: the current offset in the buffer
     * @param bytestowrite: the number of bytes to write
     */
    void write_segment(char* p, int offset, int bytestowrite)
    {
        printf("RemoteObjectImpl::write_segment(p, offset = %d, bytestowrite = %d)\n", offset, bytestowrite);
        assert(bytestowrite <= SEGMENT_LENGTH);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /**
     * @brief synchronous version
     * @param p: pointer to the buffer
     * @param offset: the current offset in the buffer
     * @param bytestoread: the number of bytes to write
     */
    bool read_segment(char* p, int offset, int bytestoread)
    {
        printf("RemoteObjectImpl::read_segment(p, offset = %d, bytestoread = %d)\n", offset, bytestoread);
        assert(bytestoread == SEGMENT_LENGTH);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m_readOffset += bytestoread;
        // We don't really read. Just check if we have read the complete buffer.
        bool read_complete = (m_readOffset > INBUFFER_LENGTH);
        return read_complete;
    }
    
    /**
     * @brief asynchronous version of write_segment
     * @param p: pointer to the buffer
     * @param offset: the current offset in the buffer
     * @param bytestowrite: the number of bytes to write
     * @param lambda: the lambda to call when the operation has completed
     */
    void sendc_write_segment(char* p, int offset, int bytestowrite, lambda_void_t lambda)
    {
        printf("RemoteObjectImpl::sendc_write_segment(p, offset = %d, bytestowrite = %d)\n", offset, bytestowrite);
        assert(bytestowrite <= SEGMENT_LENGTH);
        
        // There isn't an I/O system that will place the lambda in the event queue
        // when an I/O event arrives. Therefore we do it ourselves.
        eventQueue.push(lambda);
    }

    /**
     * @brief asynchronous version of read_segment
     * @param p: pointer to the buffer
     * @param offset: the current offset in the buffer
     * @param bytestoread: the number of bytes to write
     * @param lambda: the lambda to call when the operation has completed
     */
    void sendc_read_segment(char* p, int offset, int bytestoread, lambda_bool_t lambda)
    {
        printf("RemoteObjectImpl::sendc_read_segment(p, offset = %d, bytestoread = %d)\n", offset, bytestoread);
        assert(bytestoread == SEGMENT_LENGTH);
        m_readOffset += bytestoread;
        
        // We don't really read. Just check if we have read the complete buffer.
        bool read_complete = (m_readOffset > INBUFFER_LENGTH);
        
        // There isn't an I/O system that will place the lambda in the event queue
        // when an I/O event arrives. Therefore we do it ourselves.
        eventQueue.push([read_complete, lambda]() { lambda(read_complete); });
    }

private:
    int m_readOffset;
};

#endif
