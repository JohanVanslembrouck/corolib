/**
 * @file commclient.h
 * @brief
 * Implements the client side of an application. CommClient inherits the functionality
 * of CommCore and just adds a connect operation to it.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMCLIENT_H_
#define _COMMCLIENT_H_

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

#include "async_operation.h"
#include "commservice.h"
#include "commcore.h"

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    class CommClient : public CommCore
    {
    public:
        CommClient(boost::asio::io_context& io_context,
                   boost::asio::ip::tcp::endpoint ep);

        // Called by the user of the CommClient class to initiate the connection process.
        void start();
        async_operation<void> start_connecting();

    protected:
        void start_connect(const int idx);
        
    protected:
        boost::asio::ip::tcp::endpoint m_ep;
    };
}

#endif
