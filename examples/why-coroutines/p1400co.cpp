/**
 * @file p1400co.cpp
 * @brief 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include "p1400co.h"

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

void RemoteObjectImplCo::start_write_segment_impl(int idx, char* p, int offset, int bytestowrite)
{
    print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d)\n", this, idx);

    m_remoteObjectImpl.sendc_write_segment(p, offset, bytestowrite,
        [this, idx]()
        {
            print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImplCo::start_write_segment_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}

void RemoteObjectImplCo::start_read_segment_impl(int idx, char* p, int offset, int length)
{
    print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d)\n", this, idx);

    m_remoteObjectImpl.sendc_read_segment(p, offset, length,
        [this, idx](bool result)
        {
            print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d) - handler\n", this, idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<bool>* om_async_operation_t =
                static_cast<async_operation<bool>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d) - handler: om_async_operation_t->set_result()\n", this, idx);
                om_async_operation_t->set_result(result);
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI2, "%p: RemoteObjectImpl::start_read_segment_impl(%d) - handler: Warning: om_async_operation_t == nullptr\n", this, idx);
            }
        });
}
