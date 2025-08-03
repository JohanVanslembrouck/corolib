/**
 * @file eventqueuethr.h
 * @brief
 * QueueThr is a thread-safe variant of Queue in eventqueue.h.
 * 
 * It will be used in tutorial applications where completion takes place on a dedicated thread.
 * This dedicated completion thread places a functor onto the event queue.
 * The original launching thread pops the functors from the queue 
 * and calls the functors to resume the coroutines on the thread on which they were started.
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
 
#ifndef _EVENTQUEUETHR_H_
#define _EVENTQUEUETHR_H_

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
            std::unique_lock lock{m_mutex};
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
            std::unique_lock lock{m_mutex};
            if (m_occupiedCells == 0)
            {
                m_cv.wait(lock,
                          [&]() { return m_occupiedCells > 0; });
            }
            readValue = m_buffer[m_readIndex];
		    m_readIndex = (m_readIndex + 1) & (ARRAYSIZE - 1);
            --m_occupiedCells;
        }
        m_cv.notify_one();
        return readValue;
    }

private:
    std::condition_variable m_cv;
    std::mutex m_mutex;
    std::array<TYPE, ARRAYSIZE> m_buffer;
   
    size_t m_occupiedCells{0};
    int m_writeIndex{0};
    int m_readIndex{0};
};

#include <functional>

constexpr int ARRAYSIZE = 16;   // Use 2^N

using EventQueueThrFunctionVoidInt = QueueThr<std::function<void(int)>, ARRAYSIZE>;
using EventQueueThrFunctionVoidVoid = QueueThr<std::function<void(void)>, ARRAYSIZE>;

void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size, int val = 10);
void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size);

#endif
