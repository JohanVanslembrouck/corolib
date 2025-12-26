/**
 * @file mini_awaiter_p.h
 * @brief
 * Defines a simple awaitable.
 * class mini_awaiter can be considered to be a very simplified version of async_operation.
 * It can be used in single-threaded applications.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _MINI_AWAITER_P_H_
#define _MINI_AWAITER_P_H_

#include <stdio.h>
#include <atomic>
#include <mutex>
#include <coroutine>

#include "tracker.h"

#include "print.h"
#include "counter.h"

class mini_awaiter
{
public:
    bool await_ready() {
        counter++;
        print(PRI3, "%p: %d: mini_awaiter::await_ready() -> bool: return %d\n", this, counter, m_ready);
        return m_ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        counter++;
        print(PRI3, "%p: %d: mini_awaiter::await_suspend(...) -> void\n", this, counter);
        m_awaiting = awaiting;
    }

    int await_resume() {
        counter++;
        print(PRI3, "%p: %d: mini_awaiter::await_resume() -> int: return %d\n", this, counter, m_result);
        return m_result;
    }

    /**
     * @brief resumes the coroutine referenced by m_awaiting (if initialized)
     *
     */
    void set_result_and_resume(int result = 0) {
        counter++;
        int counter1 = counter;
        print(PRI3, "%p: %d: mini_awaiter::set_result_and_resume(%d) -> void: enter\n", this, counter, result);
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                tracker_obj.nr_resumptions++;
                counter++;
                int counter2 = counter;
                print(PRI3, "%p: %d: mini_awaiter::set_result_and_resume(%d) -> void: before m_awaiting.resume();\n", this, counter, result);
                m_awaiting.resume();
                print(PRI3, "%p: %d: mini_awaiter::set_result_and_resume(%d) -> void: after m_awaiting.resume();\n", this, counter2, result);
            }
        }
        else {
            print(PRI1, "%p: mini_awaiter::resume() could not resume() because m_awaiting == nullptr\n", this);
        }
        print(PRI3, "%p: %d: mini_awaiter::set_result_and_resume(%d) -> void: leave\n", this, counter1, result);
    }

private:
    std::coroutine_handle<> m_awaiting = nullptr;
    bool m_ready = false;
    int m_result = -1;
};

#endif
