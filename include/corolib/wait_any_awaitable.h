/**
 * @file wait_any_awaitable.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ANY_AWAITABLE_
#define _WAIT_ANY_AWAITABLE_

#include "print.h"
#include "wait_any.h"
#include "async_operation.h"

namespace corolib
{
	template<typename TYPE>
	struct wait_any_awaitable
	{
		wait_any_awaitable(std::initializer_list<TYPE*> aws)
			: m_size(aws.size())
		{
			print(PRI2, "%p: wait_any_awaitable::wait_any_awaitable(std::initializer_list<TYPE*> aws)\n", this);
			m_wait_any = new wait_any[m_size];
			int i = 0;
			for (TYPE* a : aws)
			{
				a->setWaitAny(&m_wait_any[i++]);	// defined in async_operation
			}
		}

		wait_any_awaitable(TYPE* aws, int size)
			: m_size(size)
		{
			print(PRI2, "%p: wait_any_awaitable::wait_any_awaitable(TYPE* aws, int size)\n", this);
			m_wait_any = new wait_any[m_size];
			for (int i = 0; i < size; i++)
			{
				aws[i].setWaitAny(&m_wait_any[i]);	// defined in async_operation
			}
		}

		wait_any_awaitable(const wait_any_awaitable& s) = delete;

		wait_any_awaitable(wait_any_awaitable&& s)
		{
			print(PRI2, "%p: wait_any_awaitable::wait_any_awaitable(wait_any_awaitable&& s)\n", this);
		}

		~wait_any_awaitable()
		{
			print(PRI2, "%p: wait_any_awaitable::~wait_any_awaitable()\n", this);
			delete[] m_wait_any;
		}

		wait_any_awaitable& operator = (const wait_any_awaitable&) = delete;

		wait_any_awaitable& operator = (wait_any_awaitable&& s)
		{
			print(PRI2, "%p: wait_any_awaitable::wait_any_awaitable = (wait_any_awaitable&& s)\n", this);
			s.coro = nullptr;
			return *this;
		}

		auto operator co_await() noexcept
		{
			class awaiter
			{
			public:
				awaiter(wait_any_awaitable& sync_) : m_sync(sync_) {}

				bool await_ready()
				{
					bool ready = false;
					for (int i = 0; i < m_sync.m_size; i++)
					{
						if (m_sync.m_wait_any[i].get_completed())
						{
							ready = true;
							break;
						}
					}
					print(PRI2, "%p: wait_any_awaitable::await_ready(): return %d;\n", this, ready);
					return ready;
				}

				void await_suspend(std::experimental::coroutine_handle<> awaiting)
				{
					print(PRI2, "%p: wait_any_awaitable::await_suspend(...)\n", this);
					for (int i = 0; i < m_sync.m_size; i++)
					{
						m_sync.m_wait_any[i].set_awaiting(awaiting);
					}
				}

				int await_resume()
				{
					// Find out which one has completed
					print(PRI2, "%p: wait_any_awaitable::await_resume()\n", this);
					for (int i = 0; i < m_sync.m_size; i++)
					{
						if (m_sync.m_wait_any[i].get_completed())
						{
							return i;
						}
					}
					// Error, none has completed, yet the coroutine has been resumed.
					return -1;
				}
			private:
				wait_any_awaitable& m_sync;
			};

			return awaiter{ *this };
		}

	private:
		size_t m_size;
		wait_any* m_wait_any;
	};
}

#endif
