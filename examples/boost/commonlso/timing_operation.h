/**
 * @file timing_operation.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <coroutine>

#include <boost/asio.hpp>

#include <corolib/print.h>
#include <corolib/async_operation.h>

#include "boost_context.h"

namespace corolib
{
    class timing_operation_impl
    {
    public:
        timing_operation_impl(boost_context& bc, boost::asio::steady_timer& tmr, int ms)
            : m_boost_context(bc)
            , m_tmr(tmr)
            , m_time(ms)
        {

        }

        bool try_start(async_operation_ls_base&) noexcept;
        void get_result(async_operation_ls_base&);

    private:
        boost_context& m_boost_context;
        boost::asio::steady_timer& m_tmr;
        int m_time;
    };

    class timing_operation : public async_operation_ls<timing_operation>
    {
    public:
        timing_operation(boost_context& bc, boost::asio::steady_timer& tmr, int ms)
            : m_impl(bc, tmr, ms)
        {

        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        timing_operation_impl m_impl;
    };

}
