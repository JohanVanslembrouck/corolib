/**
 * @file p1000co.cpp
 * @brief 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include "p1000co.h"

/**
 * @brief
 * @param idx
 * @param in1
 * @param in2
 */
void RemoteObject1Co::start_op1_impl(const int idx, int in1, int in2)
{
    print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    m_remoteObject.sendc_op1(in1, in2, 
        [this, idx](int out1, int out2, int ret1) 
        {
            print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<op1_ret_t>* om_async_operation_t =
                static_cast<async_operation<op1_ret_t>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                op1_ret_t op1_ret = { out1, out2, ret1 };
                om_async_operation_t->set_result(op1_ret);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}
