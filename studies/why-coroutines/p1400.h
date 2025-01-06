/**
 * @file p1400.h
 * @brief Contains a RemoteObjectImpl(ementation) class that is common to all p1400-* variants.
 * RemoteObjectImpl contains functions to write and read a segment (synchronous version)
 * or to start writing and reading a segment (asynchronous version).
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1400_H_
#define _P1400_H_

#include <assert.h>

#include "common.h"
#include "buf+msg.h"

class RemoteObjectImpl
{
public:
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
        // We don't really read. Just check if we have read the complete buffer.
        bool read_complete = (offset + bytestoread > INBUFFER_LENGTH);
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
        
        registerCB(lambda);
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
        // We don't really read. Just check if we have read the complete buffer.
        bool read_complete = (offset + bytestoread > INBUFFER_LENGTH);
        
        // There isn't an I/O system that will place the lambda in the event queue
        // when an I/O event arrives. Therefore we do it ourselves.
        eventQueue.push([read_complete, lambda]() { lambda(read_complete); });
    }
};


using lambda_msg_t = typename std::function<void(Msg)>;

class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
        : m_remoteObjImpl(remoteObjImpl)
    {
    }

    Msg op1(Msg msg)
    {
        // Write part
        Buffer writebuffer;
        printf("RemoteObject1::op1()\n");
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        int buflength = writebuffer.length();
        for (int offset = 0; offset < buflength; offset += SEGMENT_LENGTH)
        {
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            printf("RemoteObject1::op1(): calling write_segment: offset = %d\n", offset);
            m_remoteObjImpl.write_segment(writebuffer.buffer(), offset, bytestowrite);
        }

        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1::op1(): calling read_segment: offset = %d\n", offset);
            completed = m_remoteObjImpl.read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        return res;
    }

    struct op1_context
    {
        int offset = 0;
        Buffer writebuffer;
        Buffer readbuffer;
        lambda_msg_t lambda;
    };

    void sendc_op1(Msg msg, lambda_msg_t op1_cb)
    {
        printf("RemoteObject1::sendc_op1(): calling write_segment\n");

        op1_context* ctxt = new op1_context;
        ctxt->lambda = op1_cb;
        //m_lambda = op1_cb;

        // Write part
        // Marshall msg into writebuffer
        // (code not present)
        // Write the first segment
        int buflength = ctxt->writebuffer.length();
        int bytestowrite = (buflength - ctxt->offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - ctxt->offset;
        m_remoteObjImpl.sendc_write_segment(ctxt->writebuffer.buffer(), ctxt->offset, bytestowrite,
            [this, ctxt]() { this->handle_write_segment_op1(ctxt); });
        ctxt->offset += SEGMENT_LENGTH;
    }

    void handle_write_segment_op1(op1_context *ctxt)
    {
        int buflength = ctxt->writebuffer.length();
        if (ctxt->offset < buflength) {
            printf("RemoteObject1::handle_write_segment_op1(%p): calling sendc_write_segment\n", ctxt);
            int bytestowrite = (buflength - ctxt->offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - ctxt->offset;
            m_remoteObjImpl.sendc_write_segment(ctxt->writebuffer.buffer(), ctxt->offset, bytestowrite,
                [this, ctxt]() { this->handle_write_segment_op1(ctxt); });
            ctxt->offset += SEGMENT_LENGTH;
        }
        else {
            // Read part
            ctxt->offset = 0;
            printf("RemoteObject1::handle_write_segment_op1(%p): calling sendc_read_segment\n", ctxt);
            m_remoteObjImpl.sendc_read_segment(ctxt->readbuffer.buffer(), ctxt->offset, SEGMENT_LENGTH,
                [this, ctxt](bool res) { this->handle_read_segment_op1(ctxt, res); });
            ctxt->offset += SEGMENT_LENGTH;
        }
    }

    void handle_read_segment_op1(op1_context* ctxt, bool complete)
    {
        printf("RemoteObject1::handle_read_segment_op1(%p, %d): calling sendc_read_segment\n", ctxt, complete);
        Msg msg;
        if (!complete) {
            m_remoteObjImpl.sendc_read_segment(ctxt->readbuffer.buffer(), ctxt->offset, SEGMENT_LENGTH,
                [this, ctxt](bool res) { this->handle_read_segment_op1(ctxt, res); });
            ctxt->offset += SEGMENT_LENGTH;
        }
        else {
            // Unmarshall msg from buf
            // (code not present)
            // Invoke the lambda passing the result
            ctxt->lambda(msg);
            delete ctxt;
        }
    }

protected:
    RemoteObjectImpl& m_remoteObjImpl;
};

#endif
