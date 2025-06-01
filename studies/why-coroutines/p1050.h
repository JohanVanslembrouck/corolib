/**
 * @file p1050.h
 * @brief Contains a RemoteObject1 class that is common to all p10XX-* and p11XX-* variants.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1050_H_
#define _P1050_H_

#include "common.h"
#include "buf+msg.h"

class RemoteObjectImpl
{
public:
    // Synchronouus functions
    void write(char*, int bytestowrite)
    {
        printf("RemoteObjectImpl::write(p, bytestowrite = %d)\n", bytestowrite);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool read(char*, int bytestoread)
    {
        printf("RemoteObjectImpl::read(p, bytestoread = %d)\n", bytestoread);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return bytestoread;
    }

    // Asynchronouus functions
    void sendc_write(char*, int bytestowrite, lambda_void_t lambda)
    {
        printf("RemoteObjectImpl::sendc_write(p, bytestowrite = %d)\n", bytestowrite);
        registerCB(lambda);
    }

    void sendc_read(char*, int bytestoread, lambda_void_t lambda)
    {
        printf("RemoteObjectImpl::sendc_read(p, bytestoread = %d)\n", bytestoread);
        registerCB(lambda);
    }
};

/**
 * @brief RemoteObject1 contains a synchronous and asynchronous operation to communicate
 * with a remote object.
 * It is used by synchronous and asynchronous applications as well as indirectly by a coroutine application.
 *
 */
class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
        : m_remoteObjImpl(remoteObjImpl)
    {
    }

    /**
     * @brief op1 is a synchronous operation
     * @param in1
     * @param in2
     * @param out1
     * @param out2
     * @return
     */
    int op1(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1::op1(in1 = %d, in2 = %d, out1 = %d, out2 = %d)\n", in1, in2, out1, out2);
        // Marshal in1 and in2 into a buffer to write to the remote object.
        // Write the buffer to the remote object.
        // Read the response from the remote object.
        // There isn't an remote object or I/O system that will send a response. 
        // Just sleep a while to simulate waiting for the response.

        Buffer writebuffer;
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        m_remoteObjImpl.write(writebuffer.buffer(), writebuffer.length());

        Buffer readbuffer;
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until read_segment reports that the read is complete.
        m_remoteObjImpl.read(readbuffer.buffer(), readbuffer.length());
        // The reponse has arrived.
        // Unmarshal out12 and out12 from the response buffer.
        out1 = 1;
        out2 = 2;
        return in1 + in2; 
    }

    struct op1_context
    {
        Buffer writebuffer;
        Buffer readbuffer;
        void* user_context = nullptr;
    };

    /**
     * @brief asynchronous operation; asynchronous variant of op1
     * This synchronous function has been split into sendc_op1, handle_write_op1 and handle_read_op1.
     * 
     * @param context
     * @param lambda
     * @param in1
     * @param in2
     * @return
     */
    void sendc_op1(void* context, int in1, int in2, lambda_vp_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(context = %p, in1 = %d, in2 = %d, lambda)\n", context, in1, in2);

        // Create a context object and place the context passed by the application
        // in the user_context field.
        op1_context* ctxt = new op1_context;
        ctxt->user_context = context;

        // Marshal in1 and in2 into the writebuffer to write to the remote object.
        // Write the buffer to the remote object.
        // (write code is not present.)
        
        m_remoteObjImpl.sendc_write(ctxt->writebuffer.buffer(), ctxt->writebuffer.length(),
            [this, ctxt, lambda, in1, in2]() {
                this->handle_write_op1(ctxt, in1, in2, lambda);
            });
    }

    void handle_write_op1(op1_context* ctxt,  int in1, int in2, lambda_vp_3int_t lambda)
    {
        printf("RemoteObject1::handle_write_op1(ctxt = %p, in1 = %d, in2 = %d, lambda)\n", ctxt, in1, in2);

        m_remoteObjImpl.sendc_read(ctxt->readbuffer.buffer(), ctxt->readbuffer.length(),
            [this, ctxt, lambda, in1, in2]() {
                this->handle_read_op1(ctxt, in1, in2, lambda);
            });
    }

    void handle_read_op1(op1_context* ctxt, int in1, int in2, lambda_vp_3int_t lambda)
    {
        printf("RemoteObject1::handle_read_op1(ctxt = %p, in1 = %d, in2 = %d, lambda)\n", ctxt, in1, in2);

        // Unmarshall msg from m_readbuffer
        // (code not present)
 
        // Invoke the lambda passing the result
        registerCB(lambda, ctxt->user_context, in1, in2);

        // Delete the ctxt object created in sendc_op1
        delete ctxt;
    }
    

    /**
     * @brief asynchronous operation; asynchronous variant of op1
     * This synchronous function has been split into sendc_op1, handle_write_op1 and handle_read_op1.
     * Notice that this variant does not have void* context as first parameter.
     * 
     * @param in1
     * @param in2
     * @param lambda
     * @return
     */
    void sendc_op1(int in1, int in2, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(in1 = %d, in2 = %d, lambda)\n", in1, in2);

        // Create a context object and place the context passed by the application
        // in the user_context field.
        op1_context* ctxt = new op1_context;
        ctxt->user_context = nullptr;

        // Marshal in1 and in2 into the writebuffer to write to the remote object.
        // Write the buffer to the remote object.
        // (write code is not present.)

        m_remoteObjImpl.sendc_write(ctxt->writebuffer.buffer(), ctxt->writebuffer.length(),
            [this, ctxt, lambda, in1, in2]() {
                this->handle_write_op1(ctxt, in1, in2, lambda);
            });
    }

    void handle_write_op1(op1_context* ctxt, int in1, int in2, lambda_3int_t lambda)
    {
        printf("RemoteObject1::handle_write_op1(ctxt = %p, in1 = %d, in2 = %d, lambda)\n", ctxt, in1, in2);

        m_remoteObjImpl.sendc_read(ctxt->readbuffer.buffer(), ctxt->readbuffer.length(),
            [this, ctxt, lambda, in1, in2]() {
                this->handle_read_op1(ctxt, in1, in2, lambda);
            });
    }

    void handle_read_op1(op1_context* ctxt, int in1, int in2, lambda_3int_t lambda)
    {
        printf("RemoteObject1::handle_read_op1(ctxt = %p, in1 = %d, in2 = %d, lambda)\n", ctxt, in1, in2);

        // Unmarshall msg from m_readbuffer
        // (code not present)

        // Invoke the lambda passing the result
        registerCB(lambda, in1, in2);

        // Delete the ctxt object created in sendc_op1
        delete ctxt;
    }

protected:
    RemoteObjectImpl& m_remoteObjImpl;
};

#endif
