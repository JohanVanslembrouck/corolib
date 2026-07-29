/**
 * @file commclientsync.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _COMMCLIENTSYNC_H_
#define _COMMCLIENTSYNC_H_

#include <boost/asio.hpp>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>

#include "commcoresync.h"

namespace corolib
{
    using FunctionVoidVoid = std::function<void(void)>;
    using EventQueueFunctionVoidVoid = QueueThreadSafe<FunctionVoidVoid, ARRAYSIZE>;
    using CommQueue = EventQueueFunctionVoidVoid;

    class CommClient : public CommCore
    {
    public:
        explicit CommClient(boost::asio::io_context& io_context,
            CommQueue& eventqueue,
            boost::asio::ip::tcp::endpoint ep);

        async_operation<void> start_connecting();

    protected:
        boost::asio::ip::tcp::endpoint m_ep;
    };
}

#endif