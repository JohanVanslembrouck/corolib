/**
 * @file read_operation.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <corolib/async_operation.h>

#include "boost_context.h"

namespace corolib
{
    using boost::asio::steady_timer;
    using boost::asio::ip::tcp;

    class read_operation_impl
    {
    public:
        read_operation_impl(boost_context& bc, const char ch)
            : m_boost_context(bc)
            , m_ch(ch)
        {

        }

        bool try_start(async_operation_ls_base&) noexcept;
        std::string get_result(async_operation_ls_base&);

    private:
        boost_context& m_boost_context;
        const char m_ch;

        std::string m_input_buffer;
        std::string m_read_buffer;
        std::size_t m_bytes;
    };

    class read_operation : public async_operation_ls<read_operation>
    {
    public:
        read_operation(boost_context& bc, const char ch)
            : m_impl(bc, ch)
        {

        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        std::string get_result() { return m_impl.get_result(*this); }

        read_operation_impl m_impl;
    };

}
