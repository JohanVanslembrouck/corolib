/**
 * @file mini_awaiter_ts.h
 * @brief
 * Defines a simple awaitable.
 * class mini_awaiter_ts can be considered to be a very simplified version of async_operation.
 * It is a thread-safe version of class mini_awaiter in mini_awaiter.h.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _MINI_AWAITER_TS_H_
#define _MINI_AWAITER_TS_H_

#include <atomic>
#include <mutex>
#include <coroutine>

#include "tracker1.h"

class mini_awaiter_ts
{
public:
    // The await_ functions run on the "launching thread"
    bool await_ready() {
        return m_ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_awaiting = awaiting;
    }

    int await_resume() {
        return m_result;
    }

    // set_result_and_resume() runs on the completion thread
    void set_result_and_resume(int result = 0) {
        m_result = result;
        resume();
    }

protected:
    /**
     * @brief resumes the coroutine referenced by m_awaiting (if initialized)
     * runs on the completion thread
     *
     */
    void resume() {
        m_mutex.lock();
        m_ready = true;
        if (m_awaiting) {
            m_mutex.unlock();
            if (!m_awaiting.done()) {
                tracker1_obj.nr_resumptions++;
                m_awaiting.resume();
            }
        }
        else {
            m_mutex.unlock();
            printf("mini_awaiter::resume() could not resume() because m_awaiting == nullptr\n");
        }
    }


private:
    std::coroutine_handle<> m_awaiting = nullptr;
    std::atomic<bool> m_ready = false;
    std::mutex m_mutex;
    int m_result = -1;
};

#endif
