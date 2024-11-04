/**
 * @file interfacea_coroutine.cpp
 * @brief
 * See interfacea_coroutine.h for further explanation.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "interfacea_coroutine.h"

namespace moduleA
{
    async_operation<operation1_result> interfaceACo::start_operation1(CORBA::Long in_val, CORBA::Double inout_val)
    {
        printf("interfaceACo::start_operation1(in_val = %ld, inout_val = %f)\n", in_val, inout_val);
        int index = get_free_index();
        async_operation<operation1_result> ret{ this, index };
        start_operation1_impl(index, in_val, inout_val);
        return ret;
    }

    void interfaceACo::start_operation1_impl(const int idx, CORBA::Long in_val, CORBA::Double inout_val)
    {
        interfaceACoHandler_impl_ptr handler = new interfaceACoHandler_impl(this, idx);
        handler->calculate_result(in_val, inout_val);
        eventqueue.registerCB(handler);
    }

    void interfaceACoHandler_impl::operator()()
    {
        async_operation_base* om_async_operation = m_itf->get_async_operation(m_idx);
        async_operation<operation1_result>* om_async_operation_t =
            static_cast<async_operation<operation1_result>*>(om_async_operation);

        if (om_async_operation_t)
        {
            operation1_result res;
            // Retrieve the result from interfaceACoHandler_impl and place it in res.
            operation1(res);
            om_async_operation_t->set_result_and_complete(res);
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
        }

        delete this;
    }

    void interfaceACoHandler_impl::operation1(operation1_result& res)
    {
        res.m_ret_val = m_ret_val;
        res.m_inout_val = m_inout_val;
        res.m_out_val = m_out_val;
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
    void interfaceACoHandler_impl::calculate_result(CORBA::Long in_val, CORBA::Double inout_val)
    {
        m_inout_val = 2.0 * inout_val;
        m_out_val = 3 * in_val;
        m_ret_val = 2 * in_val;
    }
}
