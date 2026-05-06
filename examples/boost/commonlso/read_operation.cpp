/**
 * @file read_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "read_operation.h"

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <corolib/print.h>

namespace corolib
{
    bool read_operation_impl::try_start(async_operation_ls_base& operation) noexcept
    {
        print(PRI2, "%p: read_operation_impl::try_start()\n", this);

        boost::asio::async_read_until(
            m_boost_context.m_socket,
            boost::asio::dynamic_buffer(m_input_buffer), m_ch,
            [this, &operation](const boost::system::error_code& error,
                std::size_t bytes)
            {
                print(PRI2, "%p: read_operation_impl::handle_read(): entry\n", this);

                if (m_boost_context.m_stopped)
                {
                    print(PRI2, "%p: read_operation_impl::handle_read(): stopped\n", this);
                    return;
                }

                if (!error)
                {
                    print(PRI3, "%p: read_operation_impl::handle_read(): bytes = %d, m_input_buffer = %s\n", this, bytes, m_input_buffer.c_str());
                    m_bytes = bytes;

                    // Copy from m_input_buffer to m_read_buffer is not absolutely necessary.
                    m_read_buffer = m_input_buffer;

                    print(PRI3, "%p: read_operation_impl::handle_read(): m_bytes = %d, m_read_buffer = %s\n", this, m_bytes, m_read_buffer.c_str());
                }
                else
                {
                    m_read_buffer = "EOF";
                }

                operation.completed();
                print(PRI2, "%p: read_operation_impl::handle_read(): exit\n\n", this);
            });

        return true;
    }

    std::string read_operation_impl::get_result(async_operation_ls_base&)
    {
        print(PRI2, "%p: accept_operation_impl::get_result()\n", this);
        return m_read_buffer;
    }

}
