/**
 * @file queuethr.h
 * @brief
 * QueueThr is a thread-safe variant of Queue in eventqueue.h.
 *
 * The implementation of QueueThr is based upon class CircularBuffer (Fig. 17.9) from the book
 * C++20 for Programmers - An Objects-Natural Approach
 * Paul Deitel and Harvey Deitel
 * Copyright (C) Pearson Education, Inc.
 *
 * Notice that QueueThr uses a std::array to store its elements,
 * while Queue in eventqueue.h uses a std:queue.
 *
 * For use in applications, please use QueueThreadSafe from include/corolib/eventqueue.h
 *
 * @author Johan Vanslembrouck
 */

#ifndef _QUEUETHR_H_
#define _QUEUETHR_H_

#include <array> 
#include <condition_variable> 
#include <mutex>

template <typename TYPE, int ARRAYSIZE>
class QueueThr
{
public:
    void push(TYPE value)
    {
        {
            std::unique_lock lock{ m_mutex };
            if (m_occupiedCells == m_buffer.size())
            {
                m_cv.wait(lock,
                    [&] { return m_occupiedCells < m_buffer.size(); });
            }
            m_buffer[m_writeIndex] = value;
            ++m_occupiedCells;
            m_writeIndex = (m_writeIndex + 1) & (ARRAYSIZE - 1);
        }
        m_cv.notify_one();
    }

    TYPE pop()
    {
        TYPE readValue;
        {
            std::unique_lock lock{ m_mutex };
            if (m_occupiedCells == 0)
            {
                m_cv.wait(lock,
                    [&]() { return m_occupiedCells > 0; });
            }
            readValue = m_buffer[m_readIndex];
            m_readIndex = (m_readIndex + 1) & (ARRAYSIZE - 1);
            --m_occupiedCells;
            if (m_pushCounter > 0)
				--m_pushCounter;
        }
        m_cv.notify_one();
        return readValue;
    }

private:
    std::condition_variable m_cv;
    std::mutex m_mutex;
    std::array<TYPE, ARRAYSIZE> m_buffer;

    size_t m_occupiedCells{ 0 };
    
    int m_writeIndex{ 0 };
    int m_readIndex{ 0 };
	
    /**
     * pushCounter was not part of the original implementation of QueueThr.
     * The following explains the reason for its addition.
     * 
     * In the examples of this tutorial, push() is called from a "secondary" thread.
     * The push call places a completion handler (a functor) onto a QueueThr queue.
     * The main thread will then call function runEventQueueThr.
     * This function pops the completion handler from the queue and executes it,
     * see eventqueuethr.h and eventqueuethr.cpp.
     * Using this approach, the application code does not have to deal with multi-threaded 
     * (synchrononization) issues: the only data shared between threads is a QueueThr queue.
     * 
     * Although the secondary thread is started before we call runEventQueueThr,
     * this thread will usually be scheduled by the OS *after* we have entered runEventQueueThr.
     * In other words, the queue will still be empty at the moment we enter runEventQueueThr.
     * 
     * To inform runEventQueueThr how many elements it must pop and process before returning control to its caller,
     * we increment m_pushCounter before we start the secondary thread that pushes a completion handler.
     * This way, runEventQueueThr knows how many threads will be scheduled, i.e. how many completion handlers
     * it must pop and execute.
     * 
     */
public:
	int getPushCounter() { return m_pushCounter; }
    void incrementPushCounter() { m_pushCounter++; }
private:
	int m_pushCounter{ 0 };
};

#endif
