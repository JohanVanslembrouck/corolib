/**
 * @file commserverlso.cpp
 * @brief
 * Implements the server side of an application.
 * CommServer just implements the accept operation.
 * The communication with the connected client is then performed using a CommCore object.
 *
 * @author Johan Vanslembrouck
 */

#include <boost/asio.hpp>

#include <corolib/print.h>

#include "commserverlso.h"

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

accept_operation CommServer::start_accepting(spCommCore commRWT)
{
    print(PRI2, "%p: CommServer::start_accepting()\n", this);
    accept_operation ret(commRWT->m_boost_context, m_acceptor, m_stop);
    return ret;
}

void CommServer::stop()
{
    print(PRI2, "%p: CommServer::stop()\n", this);
    m_stop = true;
    m_acceptor.cancel();
}

}
