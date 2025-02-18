/**
 * @file mini_awaiter.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include <coroutine>

#include "corolib/print.h"

using namespace corolib;

class mini_awaiter
{
public:
    ~mini_awaiter() {
        //print(PRI1, "mini_awaiter::~mini_awaiter(...)\n");
        m_result = -2;
    }

    bool await_ready() {
        //print(PRI1, "mini_awaiter::await_ready(): m_ready = %d\n", m_ready);
        return m_ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        //print(PRI1, "mini_awaiter::await_suspend(...)\n");
        m_awaiting = awaiting;
    }

    int await_resume() {
        //print(PRI1, "mini_awaiter::await_resume(): m_result = %d\n", m_result);
        return m_result;
    }

    void set_result_and_resume(int result = 0) {
        //print(PRI1, "mini_awaiter::set_result_and_resume(%d)\n", result);
        m_result = result;
        m_ready = true;
        if (m_awaiting) {
            if (!m_awaiting.done()) {
                m_awaiting.resume();
            }
            else {
                print(PRI1, "mini_awaiter::set_result_and_resume() did not resume() because m_awaiting.done() returned true\n");
            }
        }
        else {
            print(PRI1, "mini_awaiter::set_result_and_resume() could not resume() because m_awaiting == nullptr\n");
        }
    }

protected:
    std::coroutine_handle<> m_awaiting = nullptr;
    bool m_ready = false;
    int m_result = -1;
};
