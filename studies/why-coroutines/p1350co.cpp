/**
 * @file p1350co.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "p1350co.h"

void RemoteObjectImplCo::start_write_impl(int idx, char* p, int bytestowrite)
{
    print(PRI2, "%p: RemoteObjectImplCo::start_write_impl(%d)\n", this, idx);

    m_remoteObjectImpl.sendc_write(p, bytestowrite,
        [this, idx]()
        {
            print(PRI2, "%p: RemoteObjectImplCo::start_write_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImplCo::start_write_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImplCo::start_write_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

void RemoteObjectImplCo::start_read_impl(int idx, char* p, int length)
{
    print(PRI2, "%p: RemoteObjectImpl::start_read_impl(%d)\n", this, idx);

    m_remoteObjectImpl.sendc_read(p, length,
        [this, idx]()
        {
            print(PRI2, "%p: RemoteObjectImpl::start_read_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<bool>* om_async_operation_t =
                static_cast<async_operation<bool>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImpl::start_read_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImpl::start_read_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}
