/**
 * @file eventqueue.h
 * @brief
 * class QueueThreadSafe is a thread-safe queue.
 * 
 * Its implementation is based upon class CircularBuffer (Fig. 17.9) from the book
 * C++20 for Programmers - An Objects-Natural Approach
 * Paul Deitel and Harvey Deitel
 * Copyright (C) Pearson Education, Inc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#ifndef _EVENTQUEUE_H_
#define _EVENTQUEUE_H_

#include <array> 
#include <condition_variable> 
#include <mutex>

#include <functional>

#include <stdio.h> 

namespace corolib
{
    template <typename TYPE, int ARRAYSIZE>
    class QueueThreadSafe
    {
    public:
        void push(TYPE value)
        {
            {
                std::unique_lock lock{ m_mutex };
                if (m_occupiedCells == static_cast<int>(m_buffer.size()))
                {
                    m_cv.wait(lock,
                        [&] { return m_occupiedCells < static_cast<int>(m_buffer.size()); });
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
            }
            m_cv.notify_one();
            return readValue;
        }

        void pushFinal(TYPE value)
        {
            push(value);
            m_stop = true;
        }

        void reset()
        {
            m_stop = false;
        }

        bool stopped()
        {
            return m_stop;
        }

        void printInfo()
        {
            printf("Queue\n");
            printf("    m_occupiedCells = %d\n", m_occupiedCells);
            printf("    m_writeIndex    = %d\n", m_writeIndex);
            printf("    m_readIndex     = %d\n", m_readIndex);
            printf("    m_stop          = %d\n", m_stop);
        }

        int length()
        {
            return m_occupiedCells;
        }

    private:
        std::condition_variable m_cv;
        std::mutex m_mutex;
        std::array<TYPE, ARRAYSIZE> m_buffer;

        int m_occupiedCells{ 0 };
        int m_writeIndex{ 0 };
        int m_readIndex{ 0 };
        bool m_stop{ false };
    };

    constexpr int ARRAYSIZE = 16;   // Use 2^N

    using EventQueueFunctionVoidVoid = QueueThreadSafe<std::function<void(void)>, ARRAYSIZE>;

    // Use big size that supersedes the number of events in the applications
    void runEventQueue(EventQueueFunctionVoidVoid& queue, int size = 100000);
}

#endif
