/**
 *  Filename: csemaphore.h
 *  Description:
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)/
 */

#pragma once

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

    void signal() {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait() {
        std::unique_lock < std::mutex > lock(mutex_);
        while (!count_)
            condition_.wait(lock);
        --count_;
    }
};