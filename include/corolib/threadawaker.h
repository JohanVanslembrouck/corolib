/**
 * @file threadawakter.h
 * @brief
 * class ThreadAwaker is used to allow threads that call a completion function 
 * to proceed once the thread that started the asynchronous operations has
 * reached the point where it waits for the result of the operation,
 * i.e. once all coroutines have suspended.
 * This prevents concurrency between the thread on which the coroutines suspend
 * and the thread(s) that resume the coroutines after completion of the asynchronous operation.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _THREADAWAKER_H_
#define _THREADAWAKER_H_

#include <semaphore>

#include "print.h"

namespace corolib
{
    class ThreadAwaker
    {
    public:
        void addThread()
        {
            ++m_nrThreads;
            print(PRI1, "ThreadAwaker::addThread(): m_nrThreads = %d\n", m_nrThreads);
        }

        void awaitRelease()
        {
            print(PRI1, "ThreadAwaker::awaitRelease(): m_nrThreads = %d\n", m_nrThreads);
            if (m_active)
                m_semaphore.acquire();
        }

        void releaseThreads()
        {
            print(PRI1, "ThreadAwaker::releaseThreads(): m_nrThreads = %d\n", m_nrThreads);
            m_active = false;   // Disable the use of the semaphore from now on
            m_semaphore.release(m_nrThreads);   // Release all threads that were added by calling addThread()
            m_nrThreads = 0;
        }

    private:
        std::counting_semaphore<32> m_semaphore{ 0 };
        std::atomic<bool> m_active = true;
        int m_nrThreads = 0;
    };
}

#endif
