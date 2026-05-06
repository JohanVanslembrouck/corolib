/**
 * @file commserverlso.h
 * @brief
 * Implements the server side of an application.
 * CommServer just implements the accept operation.
 * The communication with the connected client is then performed using a CommCore object.
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <boost/asio.hpp>

#include <corolib/print.h>

#include "commcorelso.h"

#include "accept_operation.h"
	
namespace corolib
{
    class CommServer
    {
    public:
        CommServer(
            boost::asio::io_context& ioContext,
            unsigned short CommService);

        accept_operation start_accepting(spCommCore commRWT);

        void stop();

    protected:
        boost::asio::io_context& m_IoContext;
        boost::asio::ip::tcp::acceptor m_acceptor;
        std::atomic_bool m_stop;
    };
}
