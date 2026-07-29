/**
 * @file commcoresync.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _COMMCORESYNC_H_
#define _COMMCORESYNC_H_

#include <boost/asio.hpp>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>

#include <corolib/eventqueue.h>
#include <corolib/threadpool.h>

#include "runeventqueue.h"

namespace corolib
{
    using FunctionVoidVoid = std::function<void(void)>;
    using EventQueueFunctionVoidVoid = QueueThreadSafe<FunctionVoidVoid, ARRAYSIZE>;
    using CommQueue = EventQueueFunctionVoidVoid;

    std::size_t completionCondition(
        std::string& buffer,
        const boost::system::error_code& /*error*/, /// let's ignore
        std::size_t bytes_transferred);

    class CommCore : public CommService
    {
    public:
        explicit CommCore(boost::asio::io_context& io_context, CommQueue& eventqueue);

        virtual ~CommCore() {}

        async_operation<void> start_writing(std::string& str);
        async_operation<std::string> start_reading();
        async_operation<void> start_timer(int ms, bool stop = false);

        void close()
        {
            m_socket.close();
        }

    protected:
        ThreadPool m_pool{ 4 };

        boost::asio::ip::tcp::socket m_socket;
        CommQueue& m_eventqueue;
    };
}

#endif
