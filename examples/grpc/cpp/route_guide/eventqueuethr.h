/**
 * @file eventqueuethr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
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

private:
    std::condition_variable m_cv;
    std::mutex m_mutex;
    std::array<TYPE, ARRAYSIZE> m_buffer;
   
    size_t m_occupiedCells{0};
    int m_writeIndex{0};
    int m_readIndex{0};
    bool m_stop{false};
};

#include <functional>

constexpr int ARRAYSIZE = 16;   // Use 2^N

using EventQueueThrFunctionVoidVoid = QueueThr<std::function<void(void)>, ARRAYSIZE>;

// Use big size that supersedes the number of events in the applications
void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size = 100000);

#endif
