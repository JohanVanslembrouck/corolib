/**
 * @file mini_awaiter.h
 * @brief
 * Defines a simple awaitable.
 * class mini_awaiter can be considered to be a very simplified version of async_operation.
 * It can be used in single-threaded applications.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _MINI_AWAITER_H_
#define _MINI_AWAITER_H_

#include <atomic>
#include <mutex>
#include <coroutine>

#include "tracker1.h"

class mini_awaiter
{
public:
    bool await_ready() {
        return m_ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        m_awaiting = awaiting;
    }

    int await_resume() {
        return m_result;
    }

    /**
     * @brief resumes the coroutine referenced by m_awaiting (if initialized)
     *
     */
    void set_result_and_resume(int result = 0) {
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                tracker1_obj.nr_resumptions++;
                m_awaiting.resume();
            }
        }
        else {
            printf("mini_awaiter::resume() could not resume() because m_awaiting == nullptr\n");
        }
    }

private:
    std::coroutine_handle<> m_awaiting = nullptr;
    bool m_ready = false;
    int m_result = -1;
};

#endif
