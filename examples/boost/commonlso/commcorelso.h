/**
 * @file commcorelso.h
 * @brief
 * Contains operations that are common to the client and server side:
 * read, write, start timers, closing, etc.
 * Uses Boost ASIO for communication.
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

#include "write_operation.h"
#include "read_operation.h"
#include "timing_operation.h"

namespace corolib
{
    class CommCore
    {
    public:
        explicit CommCore(boost::asio::io_context& io_context);
        virtual ~CommCore() {}

        void stop(bool stop = true);

        write_operation start_writing(const char* str, int size);
        read_operation start_reading(const char ch = '\n');
        timing_operation start_timer(steady_timer& timer, int ms);

        void check_deadline();

    public:
        friend class CommServer;
        boost_context m_boost_context;

        steady_timer m_deadline;
        steady_timer m_hearbeat_timer;
    };

    using spCommCore = std::shared_ptr<CommCore>;
}
