/**
 * @file csemaphore.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

# pragma once

#include <semaphore>

class CSemaphore
{
public:
    CSemaphore(unsigned int count = 0)
        : m_binsema(count)
    {
    }

    void release()
    {
        m_binsema.release();
    }

    void acquire()
    {
        m_binsema.acquire();
    }

private:
    std::binary_semaphore m_binsema;
};
