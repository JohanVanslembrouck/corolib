/**
 * @file accept_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <corolib/async_operation.h>

#include "boost_context.h"

namespace corolib
{
    class accept_operation_impl
    {
    public:
        accept_operation_impl(
            boost_context& bc,
            boost::asio::ip::tcp::acceptor& acceptor,
            std::atomic_bool& stop)
            : m_boost_context(bc)
            , m_acceptor(acceptor)
            , m_stop(stop)
        {

        }

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        boost_context& m_boost_context;
        boost::asio::ip::tcp::acceptor& m_acceptor;
        std::atomic_bool& m_stop;
    };

    class accept_operation : public async_operation_ls<accept_operation>
    {
    public:
        accept_operation(
            boost_context& bc,
            boost::asio::ip::tcp::acceptor& acceptor,
            std::atomic_bool& stop)
            : m_impl(bc, acceptor, stop)
        {

        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        accept_operation_impl m_impl;
    };

}
