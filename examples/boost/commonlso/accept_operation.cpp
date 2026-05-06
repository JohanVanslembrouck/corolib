/**
 * @file accept_operation.h
 * @brief 
 *
 * @author Johan Vanslembrouck
 */

#include "accept_operation.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio.hpp>

#include <corolib/print.h>

namespace corolib
{
    bool accept_operation_impl::try_start(async_operation_ls_base& operation) noexcept
    {
        print(PRI2, "%p: accept_operation_impl::try_start()\n", this);

        m_acceptor.async_accept(
            m_boost_context.m_socket,
            [this, &operation](const boost::system::error_code& ec)
            {
                print(PRI2, "%p; accept_operation_impl::handle_accept(): enter\n", this);

                if (m_stop)
                {
                    print(PRI2, "%p: accept_operation_impl::handle_accept(): stopped\n", this);
                    return;
                }
                if (ec)
                {
                    print(PRI2, "%p: accept_operation_impl::handle_accept(...): accept failed: %s\n", this, ec.message().c_str());
                }
                else
                {
                    operation.completed();
                }
                print(PRI2, "%p: accept_operation_impl::handle_accept(): exit\n\n", this);
            }
        );
        return true;
    }

    void accept_operation_impl::get_result(async_operation_ls_base&)
    {
        print(PRI2, "%p: accept_operation_impl::get_result()\n", this);
    }

}
