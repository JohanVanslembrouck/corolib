/**
 * @file wait_any.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ANY_H_
#define _WAIT_ANY_H_

#include <experimental/resumable>
#include "print.h"

namespace corolib
{
	struct wait_any
	{
		wait_any()
			: m_awaiting(nullptr)
			, m_completed(false)
		{
			print(PRI2, "%p: wait_any::wait_any()\n", this);
		}

		void set_awaiting(std::experimental::coroutine_handle<> awaiting)
		{
			m_awaiting = awaiting;
		}

		bool get_completed()
		{
			print(PRI2, "%p: wait_any::get_completed()\n", this);
			bool completed = m_completed;
			// reset m_completed to be ready for next completion
			m_completed = false;
			return completed;
		}

		void completed()
		{
			print(PRI2, "%p: wait_any::completed()\n", this);
			m_completed = true;
			// Resume the awaiting coroutine
			m_awaiting.resume();
		}

	private:
		std::experimental::coroutine_handle<> m_awaiting;
		bool m_completed;
	};
}

#endif
