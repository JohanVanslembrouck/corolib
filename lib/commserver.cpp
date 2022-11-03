/**
 * @file commserver.cpp
 * @brief
 * Implements the server side of an application.
 * CommServer just implements the accept operation.
 * The communication with the connected client is then performed using a CommCore object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <boost/asio.hpp>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/commcore.h>
#include <corolib/commclient.h>
#include <corolib/async_operation.h>
#include <corolib/commserver.h>

namespace corolib
{

using spCommCore = std::shared_ptr<CommCore>;

CommServer::CommServer(
    boost::asio::io_context& ioContext,
    unsigned short CommService) 
    : m_IoContext{ ioContext }
    , m_acceptor{ m_IoContext, {boost::asio::ip::tcp::v4(), CommService} }
    , m_stop{ false }
{
    print(PRI2, "%p: CommServer::CommServer(...)\n", this);
}

async_operation<void> CommServer::start_accepting(spCommCore commRWT)
{
    print(PRI2, "%p: CommServer::start_accepting()\n", this);
    int index = get_free_index_ts();
    async_operation<void> ret{ this, index, true };
    start_accepting_impl(commRWT, index);
    return ret;
}

void CommServer::stop()
{
    print(PRI2, "%p: CommServer::stop()\n", this);
    m_stop = true;
    m_acceptor.cancel();
}

void CommServer::start_accepting_impl(spCommCore commRWT, int idx)
{
    print(PRI2, "%p: CommServer::start_accepting_impl()\n", this);

    std::chrono::high_resolution_clock::time_point now = m_async_operation_info[idx].start;

    m_acceptor.async_accept(
        commRWT->m_socket,
        [this, idx, now](const boost::system::error_code& ec)
        {
            print(PRI2, "%p; CommServer::handle_accept(): idx = %d, entry\n", this, idx);

            if (m_stop)
            {
                print(PRI2, "%p: CommServer::handle_accept(): idx = %d, stopped\n", this, idx);
                return;
            }
            if (ec)
            {
                print(PRI2, "%p: CommServer::handle_accept(...): idx = %d, accept failed: %s\n", this, idx, ec.message().c_str());
            }
            else
            {
                completionHandler_ts_v(idx, now);
            }
            print(PRI2, "%p: CommServer::handle_accept(): idx = %d, exit\n\n", this, idx);
        }
    );
}

}
