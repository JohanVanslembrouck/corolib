/**
 * @file eventqueuethr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#ifndef _EVENTQUEUETHR_H_
#define _EVENTQUEUETHR_H_

#include <array> 
#include <condition_variable> 
#include <mutex>

constexpr int ARRAYSIZE = 16;   // Use 2^N

template <typename TYPE>
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

using EventQueueThrFunctionVoidInt = QueueThr<std::function<void(int)>>;
using EventQueueThrFunctionVoidVoid = QueueThr<std::function<void(void)>>;

void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size);
void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size);

#endif
