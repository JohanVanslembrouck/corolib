/**
 * @file commcore.h
 * @brief
 * Contains operations that are common to the client and server side:
 * read, write, start timers, closing, etc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMCORE_H_
#define _COMMCORE_H_

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio.hpp>

#include "async_operation.h"
#include "commservice.h"

using boost::asio::steady_timer;
using boost::asio::ip::tcp;

namespace corolib
{
	class CommCore : public CommService
	{
	public:
		explicit CommCore(boost::asio::io_context& io_context)
			: m_socket(io_context)
			, m_deadline(io_context)
			, m_hearbeat_timer(io_context)
			, m_bytes(0)
		{
			print(PRI2, "%p: CommCore::CommCore()\n", this);
		}

		virtual ~CommCore() {}

		// This function terminates all the actors to shut down the connection. It
		// may be called by the user of the CommCore class, or by the class itself in
		// response to graceful termination or an unrecoverable error.
		void stop()
		{
			print(PRI2, "%p: CommCore::stop()\n", this);
			m_stopped = true;
			boost::system::error_code ignored_error;
			m_socket.close(ignored_error);
			m_deadline.cancel();
			m_hearbeat_timer.cancel();
		}

		async_operation<void> start_writing(const char* str, int size)
		{
			index = (index + 1) & (NROPERATIONS - 1);
			print(PRI2, "%p: CommCore::start_writing(): index = %d\n", this, index);
			assert(m_async_operations[index] == nullptr);
			async_operation<void> ret{ this, index };
			start_write(index, str, size);
			return ret;
		}

		async_operation<std::string> start_reading(const char ch = '\n')
		{
			index = (index + 1) & (NROPERATIONS - 1);
			print(PRI2, "%p: CommCore::start_reading(): index = %d\n", this, index);
			assert(m_async_operations[index] == nullptr);
			async_operation<std::string> ret{ this, index };
			start_read(index, ch);
			return ret;
		}

		async_operation<void> start_timer(steady_timer& timer, int ms)
		{
			index = (index + 1) & (NROPERATIONS - 1);
			print(PRI2, "%p: CommCore::start_timer(timer, %d): index = %d\n", this, ms, index);
			assert(m_async_operations[index] == nullptr);
			async_operation<void> ret{ this, index };
			start_tmr(index, timer, ms);
			return ret;
		}

		async_operation<void> start_dummy()
		{
			index = (index + 1) & (NROPERATIONS - 1);
			print(PRI2, "%p: CommCore::start_dummy(): index = %d\n", this, index);
			assert(m_async_operations[index] == nullptr);
			async_operation<void> ret{ this, index };
			return ret;
		}

		void transfer(size_t bytes)
		{
			print(PRI2, "%p: CommCore::transfer() read %d bytes\n", this, bytes);
			m_read_buffer.resize(bytes);
			print(PRI2, "%p: CommCore::transfer(): std::copy\n", this, bytes);
			std::copy(m_input_buffer.cbegin(), m_input_buffer.cbegin() + bytes, m_read_buffer.begin());
		}

	protected:
		void start_write(const int idx, const char* str, int size)
		{
			print(PRI2, "%p: CommCore::start_write()\n", this);
			if (m_stopped)
			{
				print(PRI2, "%p: CommCore::start_write(): idx = %d, stopped\n", idx);
				return;
			}

			boost::asio::async_write(
				m_socket,
				boost::asio::buffer(str, size),
				[this, idx](const boost::system::error_code& error,
					std::size_t result_n)
				{
					(void)result_n;

					print(PRI2, "%p: CommCore::handle_write(): idx = %d, entry\n", this, idx);
					async_operation_base* om_async_operation = m_async_operations[idx];

					if (m_stopped)
					{
						print(PRI2, "%p: CommCore::handle_write(): idx = %d, stopped\n", idx);
						return;
					}

					if (!error)
					{
						print(PRI2, "%p: CommCore::handle_write(): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
						if (om_async_operation)
						{
							om_async_operation->completed();
						}
						else
						{
							// This can occur when the async_operation_base has gone out of scope.
							print(PRI1, "%p: CommCore::handle_write(): idx = %d, Warning: om_async_operation == nullptr\n", this, idx);
						}
					}
					else
					{
						print(PRI1, "%p: CommCore::handle_write(): idx = %d, Error on write: %s\n", this, idx, error.message().c_str());
						stop();
					}
					print(PRI2, "%p: CommCore::handle_write(): idx = %d, exit\n\n", this, idx);
				});
		}

		void start_read(const int idx, const char ch = '\n')
		{
			print(PRI2, "%p: CommCore::start_read()\n", this);
			m_input_buffer = "";
			//m_read_buffer = "";
			m_bytes = 0;

			// Set a deadline for the read operation.
			m_deadline.expires_after(std::chrono::seconds(10));

			boost::asio::async_read_until(
				m_socket,
				boost::asio::dynamic_buffer(m_input_buffer), ch,
				[this, idx](const boost::system::error_code& error,
					std::size_t bytes)
				{
					print(PRI2, "%p: CommCore::handle_read(): idx = %d, entry\n", this, idx);
					async_operation_base* om_async_operation = m_async_operations[idx];
					async_operation<std::string>* om_async_operation_t =
						dynamic_cast<async_operation<std::string>*>(om_async_operation);

					if (m_stopped)
					{
						print(PRI2, "%p: CommCore::handle_read(): idx = %d, stopped\n", this, idx);
						return;
					}

					if (!error)
					{
						print(PRI3, "%p: CommCore::handle_read(): idx = %d, bytes = %d, m_input_buffer = %s\n", this, idx, bytes, m_input_buffer.c_str());
						m_bytes = bytes;

						// Copy from m_input_buffer to m_read_buffer is not absolutely necessary.
						m_read_buffer = m_input_buffer;

						print(PRI3, "%p: CommCore::handle_read(): idx = %d, m_bytes = %d, m_read_buffer = %s\n", this, idx, m_bytes, m_read_buffer.c_str());

						print(PRI2, "%p: CommCore::handle_read(): idx = %d, om_async_operation_t = %p\n", this, idx, om_async_operation_t);
						if (om_async_operation_t)
						{
							om_async_operation_t->set_result(m_read_buffer);
							om_async_operation_t->completed();
						}
						else
						{
							// This can occur when the async_operation_base has gone out of scope.
							print(PRI1, "%p: CommCore::handle_read(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
						}
					}
					else
					{
						print(PRI1, "%p: CommCore::handle_read(): idx = %d, Error on read: %s\n", this, idx, error.message().c_str());
						// Inform the application level on the error so it can take action.
						if (om_async_operation_t)
						{
							om_async_operation_t->set_result("EOF");
							om_async_operation_t->completed();
						}
						else
						{
							// This can occur when the async_operation_base has gone out of scope.
							print(PRI1, "%p: CommCore::handle_read(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
						}
					}
					print(PRI2, "%p: CommCore::handle_read(): idx = %d, exit\n\n", this, idx);
				});
		}

		void start_tmr(const int idx, steady_timer& tmr, int ms)
		{
			print(PRI2, "%p: CommCore::start_tmr()\n", this);

			tmr.expires_after(std::chrono::milliseconds(ms));

			tmr.async_wait(
				[this, idx](const boost::system::error_code& error)
				{
					print(PRI2, "%p: CommCore::handle_timer(): idx = %d, entry\n", this, idx);
					async_operation_base* om_async_operation = m_async_operations[idx];

					if (m_stopped)
					{
						print(PRI2, "%p: CommCore::handle_timer(): idx = %d, stopped\n", idx);
						return;
					}

					if (!error)
					{
						print(PRI2, "%p: CommCore::handle_timer(): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
						if (om_async_operation)
						{
							om_async_operation->completed();
						}
						else
						{
							// This can occur when the async_operation_base has gone out of scope.
							print(PRI1, "%p: CommCore::handle_timer(): idx = %d, Warning: om_async_operation == nullptr\n", this, idx);
						}
					}
					else
					{
						//print(PRI1, "%p: CommCore::handle_timer(): idx = %d, Error on timer: %s\n", this, idx, error.message().c_str());
						//stop();
					}
					print(PRI2, "%p: CommCore::handle_timer(): idx = %d, exit\n\n", this, idx);
				});
		}

		std::string get_result() override
		{
			print(PRI2, "%p: CommClient::get_result()\n", this);

			// Extract the newline-delimited message from the buffer.
			std::string line(m_read_buffer.substr(0, m_bytes - 1));
			m_read_buffer.erase(0, m_bytes);

			// Empty messages are heartbeats and so ignored.
			if (!line.empty())
			{
				print(PRI2, "%p: Received: %s\n", this, line.c_str());
			}
			else
			{
				print(PRI2, "%p: Empty line received !!i\n", this);
			}
			return line;
		}

		void check_deadline()
		{
			print(PRI2, "%p: CommCore::check_deadline()\n", this);

			if (m_stopped) return;

			// Check whether the deadline has passed. We compare the deadline against
			// the current time since a new asynchronous operation may have moved the
			// deadline before this actor had a chance to run.
			if (m_deadline.expiry() <= steady_timer::clock_type::now())
			{
				// The deadline has passed. The socket is closed so that any outstanding
				// asynchronous operations are cancelled.
				m_socket.close();

				// There is no longer an active deadline. The expiry is set to the
				// maximum time point so that the actor takes no action until a new
				// deadline is set.
				m_deadline.expires_at(steady_timer::time_point::max());
			}

			// Put the actor back to sleep.
			m_deadline.async_wait(std::bind(&CommCore::check_deadline, this));
		}

	protected:
		friend class CommServer;

		bool m_stopped = false;
		boost::asio::ip::tcp::socket m_socket;

		std::string m_input_buffer;
		std::string m_read_buffer;
		std::size_t m_bytes;

		steady_timer m_deadline;
		steady_timer m_hearbeat_timer;
	};
}

#endif
