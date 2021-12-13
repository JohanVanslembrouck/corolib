/**
 * @file semaphore.h
 * @brief
 * Semaphore implemented using a mutex and a condition variable.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _CSEMAPHORE_H_
#define _CSEMAPHORE_H_

#include <mutex>
#include <condition_variable>

namespace corolib
{
    class Semaphore
    {
    public:
        Semaphore() : m_count() { }

        void reset()
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_count = 0;
        }

        void signal()
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            ++m_count;
            m_condition.notify_one();
        }

        void wait()
        {
            std::unique_lock < std::mutex > lock(m_mutex);
            while (!m_count)
                m_condition.wait(lock);
            --m_count;
        }

    private:
        std::mutex m_mutex;
        std::condition_variable m_condition;
        unsigned int m_count;
    };
}

#endif

