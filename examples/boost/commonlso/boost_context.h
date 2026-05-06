/**
 * @file boost_context.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <coroutine>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    class boost_context
    {
    public:
        boost_context(boost::asio::io_context& io_context)
            : m_socket(io_context)
            , m_deadline(io_context)
            , m_hearbeat_timer(io_context)
        {
        }

        ~boost_context() {}

    public:
        bool m_stopped = false;
        boost::asio::ip::tcp::socket m_socket;
        steady_timer m_deadline;
        steady_timer m_hearbeat_timer;
    };

}
