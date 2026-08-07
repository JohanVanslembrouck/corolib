/**
 * @file commcoresync.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>

#include "commcoresync.h"

namespace corolib
{
    std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred)
    {
        // print(PRI2, "completionCondition: bytes_transferred = %ld\n", bytes_transferred);
        if (!bytes_transferred)
        {
            return 1;
        }
        return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
    }

    CommCore::CommCore(boost::asio::io_context& io_context, CommQueue& eventqueue)
        : m_socket(io_context)
        , m_eventqueue(eventqueue)
    {
        print(PRI2, "%p: CommCore::CommCore()\n", this);
    }

    async_operation<void> CommCore::start_writing(std::string& str)
    {
        int index = get_free_index_ts();
        async_operation<void> ret{ this, index, true };
        m_pool.enqueue(
            [this, index, str]
            {
                print(PRI2, "start_reading: boost::asio::write(...);\n");
                boost::asio::write(m_socket, boost::asio::buffer(str));
                m_eventqueue.push([this, index] { completionHandler_v(index); });
            }
        );
        return ret;
    }

    async_operation<std::string> CommCore::start_reading()
    {
        int index = get_free_index_ts();
        async_operation<std::string> ret{ this, index, true };

        m_pool.enqueue(
            [this, index]
            {
                std::string answer;
                print(PRI2, "start_reading: boost::asio::read(...);\n");
                boost::asio::read(
                    m_socket,
                    boost::asio::dynamic_buffer(answer),
                    std::bind(completionCondition, std::ref(answer), std::placeholders::_1, std::placeholders::_2));
                m_eventqueue.push([this, index, answer] { completionHandler(index, answer); });
            });

        return ret;
    }

    async_operation<void> CommCore::start_timer(int ms)
    {
        int index = get_free_index_ts();
        async_operation<void> ret{ this, index, true };

        m_pool.enqueue(
            [this, index, ms]
            {
                print(PRI2, "start_timer: before this_thread::sleep_for(chrono::milliseconds(%d));\n", ms);
                this_thread::sleep_for(chrono::milliseconds(ms));
                print(PRI2, "start_timer: after this_thread::sleep_for(chrono::milliseconds(%d));\n", ms);
                m_eventqueue.push([this, index] { completionHandler_v(index); });
            }
        );

        return ret;
    }

    async_operation<void> CommCore::start_dummy()
    {
        int index = get_free_index_ts();
        async_operation<void> ret{ this, index, true };

        m_pool.enqueue(
            [this, index]
            {
                this_thread::sleep_for(chrono::milliseconds(100));
                m_eventqueue.pushFinal([this, index] { completionHandler_v(index); });
            }
        );

        return ret;
    }
}
