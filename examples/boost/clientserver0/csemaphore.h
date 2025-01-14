/**
 *  @file csemaphore.h
 *
 *  @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#define USE_CPP20SEMAPHORE 1

#if USE_CPP20SEMAPHORE

#include <semaphore>

class CSemaphore
{
public:
    CSemaphore(unsigned int count = 0)
        : m_binsema(count)
    { }

    void reset()
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

#else

#include <mutex>
#include <condition_variable>

class CSemaphore
{
private:
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned int count_;
public:
    CSemaphore() : count_() { }

    void reset() {
        std::unique_lock<std::mutex> lock(mutex_);
        count_ = 0;
    }

    void release() {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void acquire() {
        std::unique_lock < std::mutex > lock(mutex_);
        while (!count_)
            condition_.wait(lock);
        --count_;
    }
};

#endif
