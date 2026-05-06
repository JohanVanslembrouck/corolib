/**
 * @file connect_operation.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <corolib/async_operation.h>

#include "boost_context.h"

namespace corolib
{
    class connect_operation_impl
    {
    public:
        connect_operation_impl(boost_context& bc, boost::asio::ip::tcp::endpoint ep)
            : m_boost_context(bc)
            , m_ep(ep)
        {

        }

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        boost_context& m_boost_context;
        boost::asio::ip::tcp::endpoint m_ep;
    };

    class connect_operation : public async_operation_ls<connect_operation>
    {
    public:
        connect_operation(boost_context& bc, boost::asio::ip::tcp::endpoint& ep)
            : m_impl(bc, ep)
        {

        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        connect_operation_impl m_impl;
    };

}
