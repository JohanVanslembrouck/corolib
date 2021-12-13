/**
 * @file commserver.cpp
 * @brief
 * Implements the server side of an application.
 * CommServer just implements the accept operation.
 * The communication with the connected client is then performed using a CommCore object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */


#include <boost/asio.hpp>
#include "print.h"
#include "commservice.h"
#include "commcore.h"
#include "commclient.h"
#include "async_operation.h"

#include "commserver.h"

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
    index = (index + 1) & (NROPERATIONS - 1);
    assert(m_async_operations[index] == nullptr);
    async_operation<void> ret{ this, index };
    start_accept(commRWT, index);
    return ret;
}

void CommServer::stop()
{
    print(PRI2, "%p: CommServer::stop()\n", this);
    m_stop = true;
    m_acceptor.cancel();
}

void CommServer::start_accept(spCommCore commRWT, int idx)
{
    print(PRI2, "%p: CommServer::start_accept()\n", this);

    m_acceptor.async_accept(
        commRWT->m_socket,
        [this, idx](const boost::system::error_code& ec)
        {
            print(PRI2, "%p; CommServer::acceptHandler(): idx = %d, entry\n", this, idx);
            async_operation_base* om_async_operation = m_async_operations[idx];

            if (m_stop)
            {
                return;
            }
            if (ec)
            {
                print(PRI2, "%p: CommServer::acceptHandler(...): idx = %d, accept failed: %s\n", this, idx, ec.message().c_str());
            }
            else
            {
                print(PRI2, "%p: CommServer::acceptHandler(...): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
                //assert(om_async_operation != nullptr);
                if (om_async_operation)
                {
                    print(PRI2, "%p: CommServer::acceptHandler(...): idx = %d, before om_async_operation->completed();\n", this, idx);
                    om_async_operation->completed();
                    print(PRI2, "%p: CommServer::acceptHandler(...): idx = %d, after om_async_operation->completed();\n", this, idx);
                }
                else
                {
                    // This can occur when the async_operation_base has gone out of scope.
                    print(PRI1, "%p: CommServer::acceptHandler(): idx = %d, Warning: om_async_operation == nullptr\n", this, idx);
                }
            }
            print(PRI2, "%p: CommServer::acceptHandler(): idx = %d, exit\n\n", this, idx);
        }
    );
}

}
