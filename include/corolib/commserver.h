/**
 * @file commserver.h
 * @brief
 * Implements the server side of an application.
 * CommServer just implements the accept operation.
 * The communication with the connected client is then performed using a CommCore object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMSERVER_H_
#define _COMMSERVER_H_

#include <boost/asio.hpp>
#include "print.h"
#include "commservice.h"
#include "commcore.h"
#include "commclient.h"
#include "async_operation.h"

namespace corolib
{
    using spCommCore = std::shared_ptr<CommCore>;

    class CommServer : public CommService
    {
    public:
        CommServer(
            boost::asio::io_context& ioContext,
            unsigned short CommService);

        async_operation<void> start_accepting(spCommCore commRWT);

    public:
        void stop();
        void start_accept(spCommCore commRWT, int idx);

    protected:
        boost::asio::io_context& m_IoContext;
        boost::asio::ip::tcp::acceptor m_acceptor;
        std::atomic_bool m_stop;
    };
}

#endif