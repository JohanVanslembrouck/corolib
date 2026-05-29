///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Microsoft
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////
#ifndef CPPCORO_DETAIL_LINUX_HPP_INCLUDED
#define CPPCORO_DETAIL_LINUX_HPP_INCLUDED

#include <fcntl.h>
#include <linux/limits.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <utility>

#undef linux

namespace cppcoro
{
	namespace detail
	{
		namespace linux
		{
			using fd_t = int;

			enum message_type
			{
				CALLBACK_TYPE,
				RESUME_TYPE
			};

			class safe_fd
			{
			public:
				safe_fd()
					: m_fd(-1)
				{
				}

				explicit safe_fd(fd_t fd)
					: m_fd(fd)
				{
				}

				safe_fd(const safe_fd& other) = delete;

				safe_fd(safe_fd&& other) noexcept
					: m_fd(other.m_fd)
				{
					other.m_fd = -1;
				}

				~safe_fd() { close(); }

				safe_fd& operator=(safe_fd fd) noexcept
				{
					swap(fd);
					return *this;
				}

				constexpr fd_t fd() const { return m_fd; }

				/// Calls close() and sets the fd to -1.
				void close() noexcept;

				void swap(safe_fd& other) noexcept { std::swap(m_fd, other.m_fd); }

				bool operator==(const safe_fd& other) const { return m_fd == other.m_fd; }

				bool operator!=(const safe_fd& other) const { return m_fd != other.m_fd; }

				bool operator==(fd_t fd) const { return m_fd == fd; }

				bool operator!=(fd_t fd) const { return m_fd != fd; }

			private:
				fd_t m_fd;
			};

			struct message
			{
				enum message_type m_type;
				void* m_ptr;
			};

			struct io_state : linux::message
			{
				using callback_type = void(io_state* state);
				callback_type* m_callback;
				io_state(callback_type* callback) noexcept
					: m_callback(callback)
				{}
			};

			class message_queue
			{
			private:
				int m_pipefd[2];
				safe_fd m_epollfd;
				struct epoll_event m_ev;

			public:
				message_queue();
				~message_queue();
				void add_fd_watch(int fd, void* cb, uint32_t events);
				void remove_fd_watch(int fd);
				bool enqueue_message(void* message, message_type type);
				bool dequeue_message(void*& message, message_type& type, bool wait);
			};

			safe_fd create_event_fd();
			safe_fd create_timer_fd();
			safe_fd create_epoll_fd();

 			struct safe_file_data
 			{
 				safe_fd fd;
 				message_queue* mq;
 			};
		}  // namespace linux
	}      // namespace detail
}  // namespace cppcoro

#endif
