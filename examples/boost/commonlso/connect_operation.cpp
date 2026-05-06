/**
 * @file connect_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "connect_operation.h"

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio.hpp>

#include <corolib/print.h>

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
    bool connect_operation_impl::try_start(async_operation_ls_base& operation) noexcept
    {
        print(PRI2, "%p: connect_operation_impl::try_start()\n", this);

        m_boost_context.m_stopped = false;

        std::vector<boost::asio::ip::tcp::endpoint> eps;
        eps.push_back(m_ep);

        tcp::resolver::results_type::iterator endpoint_iter;

        // Set a deadline for the connect operation.
        m_boost_context.m_deadline.expires_after(std::chrono::seconds(5));

        // Start the asynchronous connect operation.
        boost::asio::async_connect(
            m_boost_context.m_socket,
            eps,
            [this, &operation](const boost::system::error_code& error,
                const tcp::endpoint& result_endpoint)
            {
                (void)result_endpoint;

                print(PRI2, "%p: CommClient::handle_connect(): entry\n", this);

                if (m_boost_context.m_stopped)
                    return;

                // The async_operation function automatically opens the socket at the start
                // of the asynchronous operation. If the socket is closed at this time then
                // the timeout handler must have run first.
                if (!m_boost_context.m_socket.is_open())
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect timed out\n", this);

                    // Try the next available endpoint.
                    //start_connecting_impl(idx);
                    try_start(operation);
                }
                // Check if the connect operation failed before the deadline expired.
                else if (error)
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connect error: %d\n", this, error);

                    // We need to close the socket used in the previous connection attempt
                    // before starting a new one.
                    m_boost_context.m_socket.close();

                    // Try the next available endpoint.
                    //start_connecting_impl(idx);
                    try_start(operation);
                }
                // Otherwise we have successfully established a connection.
                else
                {
                    print(PRI2, "%p: CommClient::handle_connect(): Connection successfully established\n", this);
                    operation.completed();
                }
                print(PRI2, "%p: CommClient::handle_connect(): exit\n\n", this);
            });

        return true;
    }

    void connect_operation_impl::get_result(async_operation_ls_base&)
    {
        print(PRI2, "%p: connect_operation_impl::get_result()\n", this);
    }

}
