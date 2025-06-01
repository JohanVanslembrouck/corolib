/**
 * @file interfacea.cpp
 * @brief
 * See interfacea.h for further explanation.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "interfacea.h"

namespace moduleA
{
    short interfaceA::operation1(CORBA::Long in_val, CORBA::Double& inout_val, CORBA::Short& out_val)
    {
        printf("interfaceA::operation1(in_val = %ld, inout_val = %f)\n", in_val, inout_val);
        interfaceAHandler_impl_ptr handler = new interfaceAHandler_impl;
        handler->calculate_result(in_val, inout_val);
        eventqueue.registerCB(handler);
        interfaceAHandler_impl_ptr handler2 = dynamic_cast<interfaceAHandler_impl_ptr>(eventqueue.runOnce());
        assert(handler == handler2);
        CORBA::Short ret_val;
        handler2->operation1(ret_val, inout_val, out_val);
        delete handler2;
        return ret_val;
    }

    void interfaceA::sendc_operation1(interfaceAHandler_impl_ptr handler, CORBA::Long in_val, CORBA::Double inout_val)
    {
        printf("interfaceA::sendc_operation1(handler = %p, in_val = %ld, inout_val = %f)\n", handler, in_val, inout_val);
        handler->calculate_result(in_val, inout_val);
        eventqueue.registerCB(handler);
    }

    PollerID interfaceA::sendp_operation1(CORBA::Long in_val, CORBA::Double inout_val)
    {
        printf("interfaceA::sendp_operation1(in_val = %ld, inout_val = %f)\n", in_val, inout_val);
        interfaceAHandler_impl_ptr handler = new interfaceAHandler_impl;
        handler->calculate_result(in_val, inout_val);
        return eventqueue.registerCB(handler);
    }

    CORBA::Boolean interfaceA::operation1Poller(PollerID pollerId, CORBA::Boolean /*blocking*/,
                    CORBA::Short& ret_val, CORBA::Double& inout_val, CORBA::Short& out_val)
    {
        //printf("interfaceA::operation1Poller(pollerId = %d, blocking = %d)\n", pollerId, blocking);
        interfaceAHandler_impl_ptr handler2 = dynamic_cast<interfaceAHandler_impl_ptr>(eventqueue.runOnce(pollerId));
        handler2->operation1(ret_val, inout_val, out_val);
        delete handler2;
        return true;
    }

    void interfaceAHandler_impl::operation1(CORBA::Short& ret_val, CORBA::Double& inout_val, CORBA::Short& out_val)
    {
        ret_val = m_ret_val;
        inout_val = m_inout_val;
        out_val = m_out_val;
    }

    /**
     * @brief interfaceACoHandler_impl::calculate_result
     * This is an auxiliary function that is usually not generated from the IDL file.
     * However, because there is no server side present in this simplified example,
     * this function calculates the out and return value(s) from the in value(s)
     * on behalf of the server side and stores it in data members for later retrieval.
     * This function can be considered to replace marshalling the in parameters
     * and sending a request to the server.
     * 
     */
    void interfaceAHandler_impl::calculate_result(CORBA::Long in_val, CORBA::Double inout_val)
    {
        m_inout_val = 2.0 * inout_val;
        m_out_val = 3 * in_val;
        m_ret_val = 2 * in_val;
    }
}
