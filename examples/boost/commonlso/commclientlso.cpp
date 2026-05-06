/**
 * @file commclientlso.cpp
 * @brief
 * Implements the client side of an application.CommClient inherits the functionality
 * of CommCore and just adds a connect operation to it.
 *
 */

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>

#include "commcorelso.h"
#include "commclientlso.h"

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    CommClient::CommClient(boost::asio::io_context& io_context,
        boost::asio::ip::tcp::endpoint ep)
        : CommCore(io_context)
        , m_ep(ep)
    {
        print(PRI2, "%p: CommClient::CommClient()\n", this);
    }

    // Called by the user of the CommClient class to initiate the connection process.
    void CommClient::start()
    {
        print(PRI2, "%p: CommClient::start()\n", this);

        start_connecting(m_ep);

        // Start the deadline actor. You will note that we're not setting any
        // particular deadline here. Instead, the connect and input actors will
        // update the deadline prior to each asynchronous operation.
        m_boost_context.m_deadline.async_wait(std::bind(&CommClient::check_deadline, this));
    }

    connect_operation CommClient::start_connecting(boost::asio::ip::tcp::endpoint ep)
    {
        connect_operation ret(m_boost_context, ep);
        return ret;
    }
    
}
