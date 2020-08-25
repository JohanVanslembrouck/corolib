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

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
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
			boost::asio::ip::tcp::endpoint ep)
			: CommCore(io_context)
			, m_ep(ep)
		{
			print(PRI2, "%p: CommClient::CommClient()\n", this);
		}

		// Called by the user of the CommClient class to initiate the connection process.
		void start()
		{
			print(PRI2, "%p: CommClient::start()\n", this);
			index = (index + 1) & (NROPERATIONS - 1);
			assert(m_async_operations[index] == nullptr);
			start_connect(index);

			// Start the deadline actor. You will note that we're not setting any
			// particular deadline here. Instead, the connect and input actors will
			// update the deadline prior to each asynchronous operation.
			m_deadline.async_wait(std::bind(&CommClient::check_deadline, this));
		}

		async_operation start_connecting()
		{
			print(PRI2, "%p: CommClient::start_connecting()\n", this);
			index = (index + 1) & (NROPERATIONS - 1);
			assert(m_async_operations[index] == nullptr);
			async_operation ret{ this, index };
			start_connect(index);
			return ret;
		}

	protected:
		void start_connect(const int idx)
		{
			print(PRI2, "%p: CommClient::start_connect()\n", this);

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
					async_operation* om_async_operation = m_async_operations[idx];

					if (m_stopped)
						return;

					// The async_operation() function automatically opens the socket at the start
					// of the asynchronous operation. If the socket is closed at this time then
					// the timeout handler must have run first.
					if (!m_socket.is_open())
					{
						print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connect timed out\n", this, idx);

						// Try the next available endpoint.
						start_connect(idx);
					}
					// Check if the connect operation failed before the deadline expired.
					else if (error)
					{
						print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connect error: %d\n", this, idx, error);

						// We need to close the socket used in the previous connection attempt
						// before starting a new one.
						m_socket.close();

						// Try the next available endpoint.
						start_connect(idx);
					}
					// Otherwise we have successfully established a connection.
					else
					{
						print(PRI2, "%p: CommClient::handle_connect(): idx = %d, Connection successfully established\n", this, idx);
						
						print(PRI2, "%p: CommClient::handle_connect(): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
						//assert(om_async_operation != nullptr);
						if (om_async_operation)
						{
							om_async_operation->completed();
						}
						else
						{
							// This can occur when the async_operation has gone out of scope.
							print(PRI1, "%p: CommCore::handle_connect(): idx = %d, Error: om_async_operation = %p\n", this, idx, om_async_operation);
						}
					}
					print(PRI2, "%p: CommClient::CommClient(): idx = %d, exit\n\n", this, idx);
				});
		}

		protected:
			boost::asio::ip::tcp::endpoint m_ep;
	};
}

#endif
