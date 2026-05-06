/**
 * @file write_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "write_operation.h"

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio.hpp>

#include <corolib/print.h>

using boost::asio::ip::tcp;

namespace corolib
{
    bool write_operation_impl::try_start(async_operation_ls_base& operation) noexcept
    {
        print(PRI2, "%p: write_operation_impl::try_start()\n", this);
 
        boost::asio::async_write(
            m_boost_context.m_socket,
            boost::asio::buffer(m_str, m_size),
            [this, &operation](const boost::system::error_code& error,
                std::size_t result_n)
            {
                (void)result_n;

                print(PRI2, "%p: write_operation_impl::handle_write(): entry\n", this);

                if (m_boost_context.m_stopped)
                {
                    print(PRI2, "%p: write_operation_impl::handle_write(): stopped\n", this);
                    return;
                }

                if (!error)
                {
                    operation.completed();
                }
                else
                {
                    print(PRI1, "%p: write_operation_impl::handle_write(): Error on write: %s\n", this, error.message().c_str());
                    //stop();
                }
                print(PRI2, "%p: write_operation_impl::handle_write(): exit\n\n", this);
            });
        return true;
    }

    void write_operation_impl::get_result(async_operation_ls_base&)
    {
        print(PRI2, "%p: write_operation_impl::get_result()\n", this);
    }

}
