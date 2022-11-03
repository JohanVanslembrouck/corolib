/**
 * @file commcore.cpp
 * @brief
 * Contains operations that are common to the client and server side:
 * read, write, start timers, closing, etc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

#include <corolib/commcore.h>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{

CommCore::CommCore(boost::asio::io_context& io_context)
    : m_socket(io_context)
    , m_bytes(0)
    , m_deadline(io_context)
    , m_hearbeat_timer(io_context)
{
    print(PRI2, "%p: CommCore::CommCore()\n", this);
}

/**
 * @brief CommCore::stop terminates all the actors to shut down the connection.
 * It may be called by the user of the CommCore class, or by the class itself in
 * response to graceful termination or an unrecoverable error.
 */
void CommCore::stop()
{
    print(PRI2, "%p: CommCore::stop()\n", this);
    m_stopped = true;
    boost::system::error_code ignored_error;
    m_socket.close(ignored_error);
    m_deadline.cancel();
    m_hearbeat_timer.cancel();
}

async_operation<void> CommCore::start_writing(const char* str, int size)
{
    int index = get_free_index_ts();
    print(PRI2, "%p: CommCore::start_writing(): index = %d\n", this, index);
    async_operation<void> ret{ this, index, true };
    start_writing_impl(index, str, size);
    return ret;
}

async_operation<std::string> CommCore::start_reading(const char ch)
{
    int index = get_free_index_ts();
    print(PRI2, "%p: CommCore::start_reading(): index = %d\n", this, index);
    async_operation<std::string> ret{ this, index, true };
    start_reading_impl(index, ch);
    return ret;
}

async_operation<void> CommCore::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index_ts();
    print(PRI2, "%p: CommCore::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index, true };
    start_timer_impl(index, timer, ms);
    return ret;
}

async_operation<void> CommCore::start_dummy()
{
    int index = get_free_index_ts();
    print(PRI2, "%p: CommCore::start_dummy(): index = %d\n", this, index);
    async_operation<void> ret{ this, index, true };
    return ret;
}

void CommCore::transfer(size_t bytes)
{
    print(PRI2, "%p: CommCore::transfer() read %d bytes\n", this, bytes);
    m_read_buffer.resize(bytes);
    print(PRI2, "%p: CommCore::transfer(): std::copy\n", this, bytes);
    std::copy(m_input_buffer.cbegin(), m_input_buffer.cbegin() + bytes, m_read_buffer.begin());
}

void CommCore::start_writing_impl(const int idx, const char* str, int size)
{
    print(PRI2, "%p: CommCore::start_writing_impl()\n", this);
    if (m_stopped)
    {
        print(PRI2, "%p: CommCore::start_writing_impl(): idx = %d, stopped\n", idx);
        return;
    }

    std::chrono::high_resolution_clock::time_point now = m_async_operation_info[idx].start;

    boost::asio::async_write(
        m_socket,
        boost::asio::buffer(str, size),
        [this, idx, now](const boost::system::error_code& error,
                         std::size_t result_n)
        {
            (void)result_n;

            print(PRI2, "%p: CommCore::handle_write(): idx = %d, entry\n", this, idx);

            if (m_stopped)
            {
                print(PRI2, "%p: CommCore::handle_write(): idx = %d, stopped\n", idx);
                return;
            }

            if (!error)
            {
                completionHandler_ts_v(idx, now);
            }
            else
            {
                print(PRI1, "%p: CommCore::handle_write(): idx = %d, Error on write: %s\n", this, idx, error.message().c_str());
                stop();
            }
            print(PRI2, "%p: CommCore::handle_write(): idx = %d, exit\n\n", this, idx);
        });
}

void CommCore::start_reading_impl(const int idx, const char ch)
{
    print(PRI2, "%p: CommCore::start_reading_impl()\n", this);
    m_input_buffer = "";
    //m_read_buffer = "";
    m_bytes = 0;

    // Set a deadline for the read operation.
    m_deadline.expires_after(std::chrono::seconds(10));

    std::chrono::high_resolution_clock::time_point now = m_async_operation_info[idx].start;

    boost::asio::async_read_until(
        m_socket,
        boost::asio::dynamic_buffer(m_input_buffer), ch,
        [this, idx, now](const boost::system::error_code& error,
                         std::size_t bytes)
        {
            print(PRI2, "%p: CommCore::handle_read(): idx = %d, entry\n", this, idx);
           
            if (m_stopped)
            {
                print(PRI2, "%p: CommCore::handle_read(): idx = %d, stopped\n", this, idx);
                return;
            }

            if (!error)
            {
                print(PRI3, "%p: CommCore::handle_read(): idx = %d, bytes = %d, m_input_buffer = %s\n", this, idx, bytes, m_input_buffer.c_str());
                m_bytes = bytes;

                // Copy from m_input_buffer to m_read_buffer is not absolutely necessary.
                m_read_buffer = m_input_buffer;

                print(PRI3, "%p: CommCore::handle_read(): idx = %d, m_bytes = %d, m_read_buffer = %s\n", this, idx, m_bytes, m_read_buffer.c_str());
            }
            else
            {
                m_read_buffer = "EOF";
            }
            completionHandler_ts<std::string>(idx, now, m_read_buffer);
            print(PRI2, "%p: CommCore::handle_read(): idx = %d, exit\n\n", this, idx);
        });
}

void CommCore::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI2, "%p: CommCore::start_timer_impl()\n", this);

    tmr.expires_after(std::chrono::milliseconds(ms));

    std::chrono::high_resolution_clock::time_point now = m_async_operation_info[idx].start;

    tmr.async_wait(
        [this, idx, now](const boost::system::error_code& error)
        {
            print(PRI2, "%p: CommCore::handle_timer(): idx = %d, entry\n", this, idx);
            
            if (m_stopped)
            {
                print(PRI2, "%p: CommCore::handle_timer(): idx = %d, stopped\n", idx);
                return;
            }

            if (!error)
            {
                completionHandler_ts_v(idx, now);
            }
            else
            {
                //print(PRI1, "%p: CommCore::handle_timer(): idx = %d, Error on timer: %s\n", this, idx, error.message().c_str());
                //stop();
            }
            print(PRI2, "%p: CommCore::handle_timer(): idx = %d, exit\n\n", this, idx);
        });
}

std::string CommCore::get_result()
{
    print(PRI2, "%p: CommClient::get_result()\n", this);

    // Extract the newline-delimited message from the buffer.
    std::string line(m_read_buffer.substr(0, m_bytes - 1));
    m_read_buffer.erase(0, m_bytes);

    // Empty messages are heartbeats and so ignored.
    if (!line.empty())
    {
        print(PRI2, "%p: Received: %s\n", this, line.c_str());
    }
    else
    {
        print(PRI2, "%p: Empty line received !!i\n", this);
    }
    return line;
}

void CommCore::check_deadline()
{
    print(PRI2, "%p: CommCore::check_deadline()\n", this);

    if (m_stopped) return;

    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (m_deadline.expiry() <= steady_timer::clock_type::now())
    {
        // The deadline has passed. The socket is closed so that any outstanding
        // asynchronous operations are cancelled.
        m_socket.close();

        // There is no longer an active deadline. The expiry is set to the
        // maximum time point so that the actor takes no action until a new
        // deadline is set.
        m_deadline.expires_at(steady_timer::time_point::max());
    }

    // Put the actor back to sleep.
    m_deadline.async_wait(std::bind(&CommCore::check_deadline, this));
}

}

