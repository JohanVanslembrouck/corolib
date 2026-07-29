/**
 * @file commcoresync.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>

#include "commclientsync.h"

namespace corolib
{
    CommClient::CommClient(boost::asio::io_context& io_context,
        EventQueueFunctionVoidVoid& eventqueue,
        boost::asio::ip::tcp::endpoint ep)
        : CommCore(io_context, eventqueue)
        , m_ep(ep)
    {
        print(PRI2, "%p: CommClient::CommClient()\n", this);
    }

    async_operation<void> CommClient::start_connecting()
    {
        int index = get_free_index_ts();
        async_operation<void> ret{ this, index, true };

        m_pool.enqueue(
            [this, index]
            {
                m_socket.connect(m_ep);
                m_eventqueue.push([this, index] { completionHandler_v(index); });
            });

        return ret;
    }
}