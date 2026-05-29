///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////
#ifndef CPPCORO_NET_SOCKET_RECV_OPERATION_HPP_INCLUDED
#define CPPCORO_NET_SOCKET_RECV_OPERATION_HPP_INCLUDED

#include <cppcoro/config.hpp>
#include <cppcoro/cancellation_token.hpp>

#include <cstdint>

#if CPPCORO_OS_WINNT
# include <cppcoro/detail/win32.hpp>
# include <cppcoro/detail/win32_overlapped_operation.hpp>
#elif CPPCORO_OS_LINUX
# include <cppcoro/detail/linux.hpp>
# include <cppcoro/detail/linux_async_operation.hpp>
#endif

namespace cppcoro::net
{
#if CPPCORO_OS_WINNT
	class socket;

	class socket_recv_operation_impl
	{
	public:

		socket_recv_operation_impl(
			socket& s,
			void* buffer,
			std::size_t byteCount) noexcept
			: m_socket(s)
			, m_buffer(buffer, byteCount)
		{}

		bool try_start(cppcoro::detail::win32_overlapped_operation_base& operation) noexcept;
		void cancel(cppcoro::detail::win32_overlapped_operation_base& operation) noexcept;

	private:

		socket& m_socket;
		cppcoro::detail::win32::wsabuf m_buffer;

	};

	class socket_recv_operation
		: public cppcoro::detail::win32_overlapped_operation<socket_recv_operation>
	{
	public:

		socket_recv_operation(
			socket& s,
			void* buffer,
			std::size_t byteCount) noexcept
			: m_impl(s, buffer, byteCount)
		{}

    public: // give access to corolib

		friend class cppcoro::detail::win32_overlapped_operation<socket_recv_operation>;

		bool try_start() noexcept { return m_impl.try_start(*this); }

    private:
		socket_recv_operation_impl m_impl;

	};

	class socket_recv_operation_cancellable
		: public cppcoro::detail::win32_overlapped_operation_cancellable<socket_recv_operation_cancellable>
	{
	public:

		socket_recv_operation_cancellable(
			socket& s,
			void* buffer,
			std::size_t byteCount,
			cancellation_token&& ct) noexcept
			: cppcoro::detail::win32_overlapped_operation_cancellable<socket_recv_operation_cancellable>(std::move(ct))
			, m_impl(s, buffer, byteCount)
		{}

    public: // give access to corolib

		friend class cppcoro::detail::win32_overlapped_operation_cancellable<socket_recv_operation_cancellable>;

		bool try_start() noexcept { return m_impl.try_start(*this); }
		void cancel() noexcept { m_impl.cancel(*this); }

    private:
		socket_recv_operation_impl m_impl;

	};
#elif CPPCORO_OS_LINUX
	class socket;

	class socket_recv_operation_impl
	{
	public:

		socket_recv_operation_impl(
			socket& s,
			void* buffer,
			std::size_t byteCount) noexcept
			: m_socket(s)
			, m_buffer(buffer)
			, m_byteCount(byteCount)
		{}

		bool try_start(cppcoro::detail::linux_async_operation_base& operation) noexcept;
		void cancel(cppcoro::detail::linux_async_operation_base& operation) noexcept;

	private:

		socket& m_socket;
 		void* m_buffer;
 		std::size_t m_byteCount;

	};

	class socket_recv_operation
		: public cppcoro::detail::linux_async_operation<socket_recv_operation>
	{
	public:

		socket_recv_operation(
			socket& s,
			void* buffer,
			std::size_t byteCount,
			cppcoro::detail::linux::message_queue* mq) noexcept
			: cppcoro::detail::linux_async_operation<socket_recv_operation>(mq)
				, m_impl(s, buffer, byteCount)
		{}

    public: // give access to corolib

		friend class cppcoro::detail::linux_async_operation<socket_recv_operation>;

		bool try_start() noexcept { return m_impl.try_start(*this); }

    private:
		socket_recv_operation_impl m_impl;

	};

	class socket_recv_operation_cancellable
		: public cppcoro::detail::linux_async_operation_cancellable<socket_recv_operation_cancellable>
	{
	public:

		socket_recv_operation_cancellable(
			socket& s,
			void* buffer,
			std::size_t byteCount,
			cppcoro::detail::linux::message_queue* mq,
			cancellation_token&& ct) noexcept
			: cppcoro::detail::linux_async_operation_cancellable<socket_recv_operation_cancellable>(mq, std::move(ct))
			, m_impl(s, buffer, byteCount)
		{}

    public: // give access to corolib

		friend class cppcoro::detail::linux_async_operation_cancellable<socket_recv_operation_cancellable>;

		bool try_start() noexcept { return m_impl.try_start(*this); }
		void cancel() noexcept { m_impl.cancel(*this); }

    private:
		socket_recv_operation_impl m_impl;

	};
#endif
}


#endif
