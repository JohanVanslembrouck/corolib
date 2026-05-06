/**
 * @file commclientlso.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include "connect_operation.h"

#include "commcorelso.h"

namespace corolib
{
    class CommClient : public CommCore
    {
    public:
        CommClient(boost::asio::io_context& io_context,
            boost::asio::ip::tcp::endpoint ep);

        // Called by the user of the CommClient class to initiate the connection process.
        void start();

        connect_operation start_connecting(boost::asio::ip::tcp::endpoint ep);

    protected:
        boost::asio::ip::tcp::endpoint m_ep;
    };
}

