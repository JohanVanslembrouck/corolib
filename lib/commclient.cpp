/**
 * @file commclient.cpp
 * @brief
 * Implements the client side of an application. CommClient inherits the functionality
 * of CommCore and just adds a connect operation to it.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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
#include <corolib/commcore.h>
#include <corolib/commclient.h>

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
    int index = get_free_index();
    assert(m_async_operations[index] == nullptr);
    start_connecting_impl(index);

    // Start the deadline actor. You will note that we're not setting any
    // particular deadline here. Instead, the connect and input actors will
    // update the deadline prior to each asynchronous operation.
    m_deadline.async_wait(std::bind(&CommClient::check_deadline, this));
}

async_operation<void> CommClient::start_connecting()
{
    print(PRI2, "%p: CommClient::start_connecting()\n", this);
    int index = get_free_index();
    async_operation<void> ret{ this, index };
    start_connecting_impl(index);
    return ret;
}

void CommClient::start_connecting_impl(const int idx)
{
    print(PRI2, "%p: CommClient::start_connecting_impl()\n", this);

    m_stopped = false;

    std::vector<boost::asio::ip::tcp::endpoint> eps;
    eps.push_back(m_ep);

    tcp::resolver::results_type::iterator endpoint_iter;

    // Set a deadline for the connect operation.
    m_deadline.expires_after(std::chrono::seconds(5));

    // Start the asynchronous connect operation.
    boost::asio::async_connect(
        m_socket,
        eps,
        [this, idx](const boost::system::error_code& error,
                    const tcp::endpoint& result_endpoint)
        {
            (void)result_endpoint;

            print(PRI2, "%p: CommClient::handle_connect(): idx = %d, entry\n", this, idx);

            if (m_stopped)
                return;

            // The async_operation function automatically opens the socket at the start
            // of the asynchronous operation. If the socket is closed at this time then
            // the timeout handler must have run first.
            if (!m_socket.is_open())
            {
                print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connect timed out\n", this, idx);

                // Try the next available endpoint.
                start_connecting_impl(idx);
            }
            // Check if the connect operation failed before the deadline expired.
            else if (error)
            {
                print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connect error: %d\n", this, idx, error);

                // We need to close the socket used in the previous connection attempt
                // before starting a new one.
                m_socket.close();

                // Try the next available endpoint.
                start_connecting_impl(idx);
            }
            // Otherwise we have successfully established a connection.
            else
            {
                print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connection successfully established\n", this, idx);
                
                async_operation_base* om_async_operation = m_async_operations[idx];
                print(PRI2, "%p: CommClient::handle_connect(): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
                if (om_async_operation)
                {
                    om_async_operation->completed();
                }
                else
                {
                    // This can occur when the async_operation_base has gone out of scope.
                    print(PRI1, "%p: CommCore::handle_connect(): idx = %d, Warning: om_async_operation == nullptr\n", this, idx);
                }
            }
            print(PRI2, "%p: CommClient::handle_connect(): idx = %d, exit\n\n", this, idx);
        });
}

}
