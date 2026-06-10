///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////
#ifndef CPPCORO_DETAIL_LINUX_ASYNC_OPERATION_HPP_INCLUDED
#define CPPCORO_DETAIL_LINUX_ASYNC_OPERATION_HPP_INCLUDED

#include <cppcoro/cancellation_registration.hpp>
#include <cppcoro/cancellation_token.hpp>
#include <cppcoro/operation_cancelled.hpp>

#include <cppcoro/detail/linux.hpp>

#include <optional>
#include <system_error>
#include <cppcoro/coroutine.hpp>
#include <cassert>

namespace cppcoro
{
	namespace detail
	{
		class linux_async_operation_base
			: protected detail::linux::io_state
		{
		public:
            // Addition for corolib
            using corolib_functor_t = std::function<void(std::int32_t, std::int32_t)>;

			linux_async_operation_base(
				detail::linux::io_state::callback_type* callback,
				detail::linux::message_queue* mq) noexcept
				: detail::linux::io_state(callback)
				, m_mq(mq)
			{}

			std::size_t get_result()
			{
				if (m_res < 0)
				{
					throw std::system_error{
						-m_res,
						std::system_category()
					};
				}

				return m_res;
			}

			std::int32_t m_res;
            std::int32_t m_err{};                       // Addition for corolib
			std::function<int()> m_completeFunc;
 			detail::linux::message_queue* m_mq;

            // Addition for corolib (begin)
            corolib_functor_t m_corolib_cb{};
            bool m_use_corolib{ false };
            // Addition for corolib (end)
		};

		template<typename OPERATION>
		class linux_async_operation
			: protected linux_async_operation_base
		{
		protected:

			linux_async_operation(detail::linux::message_queue* mq) noexcept
				: linux_async_operation_base(
					&linux_async_operation::on_operation_completed, mq)
			{}

		public:

			bool await_ready() const noexcept { return false; }

			CPPCORO_NOINLINE
			bool await_suspend(cppcoro::coroutine_handle<> awaitingCoroutine)
			{
				static_assert(std::is_base_of_v<linux_async_operation, OPERATION>);

				m_awaitingCoroutine = awaitingCoroutine;
				return static_cast<OPERATION*>(this)->try_start();
			}

			decltype(auto) await_resume()
			{
				return static_cast<OPERATION*>(this)->get_result();
			}

            // Addition for corolib (begin)
            void get_results(std::int32_t& errorCode, std::int32_t& numberOfBytesTransferred)
            {
                errorCode = m_err;
                numberOfBytesTransferred = m_res;
            }

            void register_corolib_cb(corolib_functor_t&& corolib_cb)
            {
                m_use_corolib = true;
                m_corolib_cb = std::move(corolib_cb);
            }
            // Addition for corolib (end)

		private:

			static void on_operation_completed(
				detail::linux::io_state* ioState) noexcept
			{
				auto* operation = static_cast<linux_async_operation*>(ioState);
				operation->m_res = operation->m_completeFunc();
				if (operation->m_res < 0) {
					operation->m_res = -errno;
                    operation->m_err = -errno;      // Addition for corolib
				}

                // Addition for corolib (begin)
                if (operation->m_use_corolib)
                {
                    if (operation->m_corolib_cb)
                    {
                        operation->m_corolib_cb(operation->m_err, operation->m_res);
                    }
                }
                else
                {
                    operation->m_awaitingCoroutine.resume();
                }
                // Addition for corolib (end)
			}

			cppcoro::coroutine_handle<> m_awaitingCoroutine;

		};

		static constexpr int error_operation_cancelled = ECANCELED;
		template<typename OPERATION>
		class linux_async_operation_cancellable
			: protected linux_async_operation_base
		{

		protected:

			linux_async_operation_cancellable(
				detail::linux::message_queue* mq,
				cancellation_token&& ct) noexcept
				: linux_async_operation_base(
					  &linux_async_operation_cancellable::on_operation_completed, mq)
				, m_state(ct.is_cancellation_requested() ? state::completed : state::not_started)
				, m_cancellationToken(std::move(ct))
			{
				m_res = -error_operation_cancelled;
			}

			linux_async_operation_cancellable(
				linux_async_operation_cancellable&& other) noexcept
				: linux_async_operation_base(std::move(other))
				, m_state(other.m_state.load(std::memory_order_relaxed))
				, m_cancellationToken(std::move(other.m_cancellationToken))
			{
				assert(m_res == other.m_res);
			}

		public:

			bool await_ready() const noexcept
			{
				return m_state.load(std::memory_order_relaxed) == state::completed;
			}

			CPPCORO_NOINLINE
			bool await_suspend(cppcoro::coroutine_handle<> awaitingCoroutine)
			{
				static_assert(std::is_base_of_v<linux_async_operation_cancellable, OPERATION>);

				m_awaitingCoroutine = awaitingCoroutine;

				// TRICKY: Register cancellation callback before starting the operation
				// in case the callback registration throws due to insufficient
				// memory. We need to make sure that the logic that occurs after
				// starting the operation is noexcept, otherwise we run into the
				// problem of not being able to cancel the started operation and
				// the dilemma of what to do with the exception.
				//
				// However, doing this means that the cancellation callback may run
				// prior to returning below so in the case that cancellation may
				// occur we defer setting the state to 'started' until after
				// the operation has finished starting. The cancellation callback
				// will only attempt to request cancellation of the operation with
				// CancelIoEx() once the state has been set to 'started'.
				const bool canBeCancelled = m_cancellationToken.can_be_cancelled();
				if (canBeCancelled)
				{
					m_cancellationCallback.emplace(
						std::move(m_cancellationToken),
						[this] { this->on_cancellation_requested(); });
				}
				else
				{
					m_state.store(state::started, std::memory_order_relaxed);
				}

				// Now start the operation.
				const bool willCompleteAsynchronously = static_cast<OPERATION*>(this)->try_start();
				if (!willCompleteAsynchronously)
				{
					// Operation completed synchronously, resume awaiting coroutine immediately.
					return false;
				}

				if (canBeCancelled)
				{
					// Need to flag that the operation has finished starting now.

					// However, the operation may have completed concurrently on
					// another thread, transitioning directly from not_started -> complete.
					// Or it may have had the cancellation callback execute and transition
					// from not_started -> cancellation_requested. We use a compare-exchange
					// to determine a winner between these potential racing cases.
					state oldState = state::not_started;
					if (!m_state.compare_exchange_strong(
						oldState,
						state::started,
						std::memory_order_release,
						std::memory_order_acquire))
					{
						if (oldState == state::cancellation_requested)
						{
							// Request the operation be cancelled.
							// Note that it may have already completed on a background
							// thread by now so this request for cancellation may end up
							// being ignored.
							static_cast<OPERATION*>(this)->cancel();

							if (!m_state.compare_exchange_strong(
								oldState,
								state::started,
								std::memory_order_release,
								std::memory_order_acquire))
							{
								assert(oldState == state::completed);
								return false;
							}
						}
						else
						{
							assert(oldState == state::completed);
							return false;
						}
					}
				}

				return true;
			}

			decltype(auto) await_resume()
			{
				// Free memory used by the cancellation callback now that the operation
				// has completed rather than waiting until the operation object destructs.
				// eg. If the operation is passed to when_all() then the operation object
				// may not be destructed until all of the operations complete.
				m_cancellationCallback.reset();

				if (m_res == -error_operation_cancelled)
				{
					throw operation_cancelled{};
				}

				return static_cast<OPERATION*>(this)->get_result();
			}

            // Addition for corolib (begin)
            // TRICKY
            // When using corolib, co_await is applied on an async_operation object,
            // not on an object of a class that is derived from linux_async_operation_cancellable,
            // such as socket_accept_operation_cancellable.
            // This means that the 3 functions above, i.e. async_ready(), async_suspend() and async_resume()
            // will not be called.
            // On compledtion, function on_operation_completed() will be called.
            // However, this function will only resume a coroutine if state == state::started.
            // Initially, state == state::not_started.
            // The state is changed to state::started in async_suspend(), but because
            // this function is not called when using corolib, the coroutine will not be resumed.
            // For this reason, the function initialize() has been introduced to change
            // the state to state::started.
            // This simple implementation is (currently) sufficient for running the examples.
            void initialize()
            {
                m_state.store(state::started, std::memory_order_relaxed);
            }

            void get_results(std::int32_t& errorCode, std::int32_t& numberOfBytesTransferred)
            {
                errorCode = m_err;
                numberOfBytesTransferred = m_res;
            }

            void register_corolib_cb(corolib_functor_t&& corolib_cb)
            {
                m_use_corolib = true;
                m_corolib_cb = std::move(corolib_cb);
            }
            // Addition for corolib (end)

		private:

			enum class state
			{
				not_started,
				started,
				cancellation_requested,
				completed
			};

			void on_cancellation_requested() noexcept
			{
				auto oldState = m_state.load(std::memory_order_acquire);
				if (oldState == state::not_started)
				{
					// This callback is running concurrently with await_suspend().
					// The call to start the operation may not have returned yet so
					// we can't safely request cancellation of it. Instead we try to
					// notify the await_suspend() thread by transitioning the state
					// to state::cancellation_requested so that the await_suspend()
					// thread can request cancellation after it has finished starting
					// the operation.
					const bool transferredCancelResponsibility =
						m_state.compare_exchange_strong(
							oldState,
							state::cancellation_requested,
							std::memory_order_release,
							std::memory_order_acquire);
					if (transferredCancelResponsibility)
					{
						return;
					}
				}

				// No point requesting cancellation if the operation has already completed.
				if (oldState != state::completed)
				{
					static_cast<OPERATION*>(this)->cancel();
				}
				m_mq->enqueue_message(reinterpret_cast<void*>(m_awaitingCoroutine.address()),
					detail::linux::RESUME_TYPE);
			}

			static void on_operation_completed(
				detail::linux::io_state* ioState) noexcept
			{
				auto* operation = static_cast<linux_async_operation_cancellable*>(ioState);

				operation->m_res = operation->m_completeFunc();
				if (operation->m_res < 0) {
					operation->m_res = -errno;
                    operation->m_err = -errno;      // Addition for corolib
				}

				auto state = operation->m_state.load(std::memory_order_acquire);
 
				if (state == state::started)
				{
					operation->m_state.store(state::completed, std::memory_order_relaxed);
                    // Addition for corolib (begin)
                    if (operation->m_use_corolib)
                    {
                        if (operation->m_corolib_cb)
                        {
                            operation->m_corolib_cb(operation->m_err, operation->m_res);
                        }
                    }
                    else
                    {
					    operation->m_awaitingCoroutine.resume();
                    }
                    // Addition for corolib (end)
				}
				else
				{
					// We are racing with await_suspend() call suspending.
					// Try to mark it as completed using an atomic exchange and look
					// at the previous value to determine whether the coroutine suspended
					// first (in which case we resume it now) or we marked it as completed
					// first (in which case await_suspend() will return false and immediately
					// resume the coroutine).
					state = operation->m_state.exchange(
						state::completed,
						std::memory_order_acq_rel);
					if (state == state::started)
					{
						// The await_suspend() method returned (or will return) 'true' and so
						// we need to resume the coroutine.
                        // Addition for corolib (begin)
                        if (operation->m_use_corolib)
                        {
                            if (operation->m_corolib_cb)
                            {
                                operation->m_corolib_cb(operation->m_err, operation->m_res);
                            }
                        }
                        else
                        {
                            operation->m_awaitingCoroutine.resume();
                        }
                        // Addition for corolib (end)
					}
				}
			}

			std::atomic<state> m_state;
			cppcoro::cancellation_token m_cancellationToken;
			std::optional<cppcoro::cancellation_registration> m_cancellationCallback;
			cppcoro::coroutine_handle<> m_awaitingCoroutine;

		};
	}
}

#endif
