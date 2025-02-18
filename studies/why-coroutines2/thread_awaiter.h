/**
 * @file thread_awaiter.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

#include "csemaphore.h"

using namespace corolib;

class thread_awaiter
{
public:
    ~thread_awaiter() {
        print(PRI1, "thread_awaiter::~thread_awaiter(...)\n");
        m_result = -2;
    }

    void set_result_and_release(int result = 0) {
        print(PRI1, "thread_awaiter::set_result_and_release(%d)\n", result);
        m_result = result;
        m_sema.release();
    }

    int get_result() {
        m_sema.acquire();
        print(PRI1, "thread_awaiter::get_result(): returns %d\n", m_result);
        return m_result;
    }

protected:
    int m_result = -1;
    CSemaphore m_sema;
};
