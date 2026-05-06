/**
 * @file commcorelso.cpp
 * @brief
 * Contains operations that are common to the client and server side:
 * read, write, start timers, closing, etc.
 *
 * @author Johan Vanslembrouck
 */
 
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

#include "commcorelso.h"

#include <corolib/print.h>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    CommCore::CommCore(boost::asio::io_context& io_context)
        : m_boost_context(io_context)
        , m_deadline(io_context)
        , m_hearbeat_timer(io_context)
    {
        print(PRI2, "%p: CommCore::CommCore()\n", this);
    }

    void CommCore::stop(bool stop)
    {
        print(PRI2, "%p: CommCore::stop()\n", this);
        if (stop)
            m_boost_context.m_stopped = true;
        boost::system::error_code ignored_error;
        m_boost_context.m_socket.close(ignored_error);
        m_deadline.cancel();
        m_hearbeat_timer.cancel();
    }

    write_operation CommCore::start_writing(const char* str, int size)
    {
        write_operation ret(m_boost_context, str, size);
        return ret;
    }

    read_operation CommCore::start_reading(const char ch)
    {
        read_operation ret(m_boost_context, ch);
        return ret;
    }

    timing_operation CommCore::start_timer(steady_timer& timer, int ms)
    {
        timing_operation ret(m_boost_context, timer, ms);
        return ret;
    }

    void CommCore::check_deadline()
    {
        print(PRI2, "%p: CommCore::check_deadline()\n", this);

        if (m_boost_context.m_stopped) return;

        // Check whether the deadline has passed. We compare the deadline against
        // the current time since a new asynchronous operation may have moved the
        // deadline before this actor had a chance to run.
        if (m_deadline.expiry() <= steady_timer::clock_type::now())
        {
            // The deadline has passed. The socket is closed so that any outstanding
            // asynchronous operations are cancelled.
            m_boost_context.m_socket.close();

            // There is no longer an active deadline. The expiry is set to the
            // maximum time point so that the actor takes no action until a new
            // deadline is set.
            m_deadline.expires_at(steady_timer::time_point::max());
        }

        // Put the actor back to sleep.
        m_deadline.async_wait(std::bind(&CommCore::check_deadline, this));
    }

}
