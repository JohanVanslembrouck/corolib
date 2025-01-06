/**
 * @file p1350.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1350_H_
#define _P1350_H_

#include "common.h"
#include "buf+msg.h"

class RemoteObjectImpl
{
public:
    // Synchronouus functions
    void write(char* p, int bytestowrite)
    {
        printf("RemoteObjectImpl::write(p, bytestowrite = %d)\n", bytestowrite);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool read(char* p, int bytestoread)
    {
        printf("RemoteObjectImpl::read(p, bytestoread = %d)\n", bytestoread);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return bytestoread;
    }

    // Asynchronouus functions
    void sendc_write(char* p, int bytestowrite, lambda_void_t lambda)
    {
        printf("RemoteObjectImpl::sendc_write(p, bytestowrite = %d)\n", bytestowrite);
        registerCB(lambda);
    }

    void sendc_read(char* p, int bytestoread, lambda_void_t lambda)
    {
        printf("RemoteObjectImpl::sendc_read(p, bytestoread = %d)\n", bytestoread);

        // There isn't an I/O system that will place the lambda in the event queue
        // when an I/O event arrives. Therefore we do it ourselves.
        eventQueue.push([lambda]() { lambda(); });
    }
};

class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
        : m_remoteObjImpl(remoteObjImpl)
    {
    }

    int op1(Msg& msg)
    {
        printf("RemoteObject1::op1(msg): msg.m_length = % d\n", msg.m_length);
        Buffer writebuffer;
        m_remoteObjImpl.write(writebuffer.buffer(), msg.m_length);
        Buffer readbuffer;
        m_remoteObjImpl.read(readbuffer.buffer(), 4);
        return 0;
    }
    
    struct op1_context
    {
        Buffer writebuffer;
        Buffer readbuffer;
    };

    void sendc_op1(Msg& msg, lambda_void_t lambda)
    {
        printf("RemoteObject1::sendc_op1(msg, lambda): msg.m_length = %d\n", msg.m_length);

        op1_context* ctxt = new op1_context;

        m_remoteObjImpl.sendc_write(ctxt->writebuffer.buffer(), msg.m_length,
            [this, ctxt, lambda]() {
                this->handle_write_op1(ctxt, lambda);
            });
    }

    void handle_write_op1(op1_context* ctxt, lambda_void_t lambda)
    {
        printf("RemoteObject1::handle_write_op1(ctxt = %p, lambda)\n", ctxt);

        m_remoteObjImpl.sendc_read(ctxt->readbuffer.buffer(), 4,
            [this, ctxt, lambda]() {
                this->handle_read_op1(ctxt, lambda);
            });
    }

    void handle_read_op1(op1_context* ctxt, lambda_void_t lambda)
    {
        printf("RemoteObject1::handle_read_op1(ctxt = %p, lambda)\n", ctxt);

        // Unmarshall msg from m_readbuffer
        // (code not present)

        // Invoke the lambda passing the result
        registerCB(lambda);

        // Delete the ctxt object created in sendc_op1
        delete ctxt;
    }

protected:
    RemoteObjectImpl& m_remoteObjImpl;
};

#endif
