/**
 * @file commcore.h
 * @brief
 * Contains operations that are common to the client and server side:
 * read, write, start timers, closing, etc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMCORE_H_
#define _COMMCORE_H_

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

#include "async_operation.h"
#include "commservice.h"

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    class CommCore : public CommService
    {
    public:
        explicit CommCore(boost::asio::io_context& io_context);
        virtual ~CommCore() {}

        // This function terminates all the actors to shut down the connection. It
        // may be called by the user of the CommCore class, or by the class itself in
        // response to graceful termination or an unrecoverable error.
        void stop();

        async_operation<void> start_writing(const char* str, int size);
        async_operation<std::string> start_reading(const char ch = '\n');
        async_operation<void> start_timer(steady_timer& timer, int ms);
        async_operation<void> start_dummy();

        void transfer(size_t bytes);

    protected:
        void start_write(const int idx, const char* str, int size);
        void start_read(const int idx, const char ch = '\n');
        void start_tmr(const int idx, steady_timer& tmr, int ms);
        std::string get_result() override;
        void check_deadline();

    protected:
        friend class CommServer;

        bool m_stopped = false;
        boost::asio::ip::tcp::socket m_socket;

        std::string m_input_buffer;
        std::string m_read_buffer;
        std::size_t m_bytes;

        steady_timer m_deadline;
        steady_timer m_hearbeat_timer;
    };
}

#endif
