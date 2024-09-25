/**
 * @file p1300co.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "p1300co.h"

void RemoteObject1Co::start_op1_impl(const int idx, Msg& msg)
{
    print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    m_remoteObject.sendc_op1(msg, 
        [this, idx]()
        {
            print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<int>* om_async_operation_t =
                static_cast<async_operation<int>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->set_result(1);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: RemoteObj1::start_op1_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}
