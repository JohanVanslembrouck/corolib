/**
 * @file wait_all_counter.h
 * @brief
 * wait_all_counter is passed a counter to decrement and the coroutine_handle of an coroutine
 * it has to resume when the counter drops to 0.
 * The same wait_all_counter object is passed to several async_operation objects
 * that each decrements the counter when the asynchronous operation they wait for completes.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ALL_COUNTER_H_
#define _WAIT_ALL_COUNTER_H_

#include <experimental/resumable>
#include "print.h"

namespace corolib
{
	struct wait_all_counter
	{
		wait_all_counter(int nr)
			: m_awaiting(nullptr)
			, m_nr(nr)
		{
			print(PRI2, "%p: wait_all_counter::wait_all_counter(%d)\n", this, nr);
		}

		void set_awaiting(std::experimental::coroutine_handle<> awaiting)
		{
			m_awaiting = awaiting;
		}

		int get_counter()
		{
			return m_nr;
		}

		void completed()
		{
			print(PRI2, "%p: wait_all_counter::completed(): m_nr = %d\n", this, m_nr);
			m_nr--;
			if (m_nr == 0)
			{
				print(PRI2, "%p: wait_all_counter::completed(): all replies received\n", this);
				m_awaiting.resume();
			}
		}

	private:
		std::experimental::coroutine_handle<> m_awaiting;
		int m_nr;
	};
}

#endif
