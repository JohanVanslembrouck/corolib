/**
 * @file async_queue.h
 * @brief async_queue is the coroutine version of a queue using a condition_variable + mutex
 * to protect its push and pop operations in a multi-threaded application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _ASYNC_QUEUE_H_
#define _ASYNC_QUEUE_H_

#include <array>

#include "commservice.h"
#include "async_operation.h"

using namespace corolib;

namespace corolib
{
    template <typename TYPE, int ARRAYSIZE>
    class async_queue : private CommService
    {
    public:

        async_operation<void> push(TYPE value)
        {
            int index = get_free_index();
            async_operation<void> ret{ this, index };

            if (m_occupiedCells == m_buffer.size())
            {
                print(PRI2, "async_queue<...>::push(): buffer full: saving value to push: %d\n", value);
                m_value_to_be_pushed = value;
                m_push_operation_index = index;
            }
            else
            {
                m_buffer[m_writeIndex] = value;
                ++m_occupiedCells;
                m_writeIndex = (m_writeIndex + 1) & (ARRAYSIZE - 1);

                if (m_pop_operation_index != -1)
                {
                    // A coroutine is co_await-ing pop()
                    int idx = m_pop_operation_index;
                    m_pop_operation_index = -1;
                    print(PRI2, "async_queue<...>::push(%d): before complete_pop(%d)\n", value, idx);
                    complete_pop(idx);
                    print(PRI2, "async_queue<...>::push(%d): after complete_pop(%d)\n", value, idx);
                }
                print(PRI2, "async_queue<...>::push(%d): ret.completed();\n", value);
                ret.completed();
            }
            return ret;
        }

        async_operation<TYPE> pop()
        {
            int index = get_free_index();
            async_operation<TYPE> ret{ this, index };

            TYPE readValue;
            if (m_occupiedCells == 0)
            {
                print(PRI2, "async_queue<...>::pop(): buffer empty: cannot pop anymore\n");
                m_pop_operation_index = index;
            }
            else
            {
                readValue = m_buffer[m_readIndex];
                m_readIndex = (m_readIndex + 1) & (ARRAYSIZE - 1);
                --m_occupiedCells;

                if (m_push_operation_index != -1)
                {
                    // A coroutine is co_await-ing push()
                    int idx = m_push_operation_index;
                    m_push_operation_index = -1;

                    print(PRI2, "async_queue<...>::pop(): before complete_push(%d)\n", idx);
                    complete_push(idx);
                    print(PRI2, "async_queue<...>::pop(): after complete_push(%d)\n", idx);
                }

                print(PRI2, "async_queue<...>::pop(): ret.set_result_and_complete(%d);\n", readValue);
                ret.set_result_and_complete(readValue);
            }
            return ret;
        }

    protected:

        /**
         * @brief complete_push() is called from pop() if m_push_operation_index != -1
         *
         */
        void complete_push(int idx)
        {
            print(PRI1, "async_queue<...>::complete_push(idx = %d): m_value_to_be_pushed = %d\n", idx, m_value_to_be_pushed);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "async_queue<...>::complete_push(%d): om_async_operation_t = %p\n", idx, om_async_operation_t);
                print(PRI2, "async_queue<...>::complete_push(%d): m_writeIndex = %d\n", idx, m_writeIndex);
                print(PRI2, "async_queue<...>::complete_push(%d): m_value_to_be_pushed = %d\n", idx, m_value_to_be_pushed);

                m_buffer[m_writeIndex] = m_value_to_be_pushed;
                ++m_occupiedCells;
                m_writeIndex = (m_writeIndex + 1) & (ARRAYSIZE - 1);

                om_async_operation_t->completed();
            }
            else
            {
                print(PRI1, "async_queue<...>::complete_push(idx = %d): om_async_operation_t = nullptr\n", idx);
            }
        }

        /**
        * @brief complete_pop() is called from push() if m_pop_operation_index != -1
        *
        */
        void complete_pop(int idx)
        {
            print(PRI1, "async_queue<...>::complete_pop(%d)\n", idx);

            TYPE readValue;

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<TYPE>* om_async_operation_t =
                dynamic_cast<async_operation<TYPE>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "async_queue<...>::complete_pop(%d): om_async_operation_t = %p\n", idx, om_async_operation_t);
                print(PRI2, "async_queue<...>::complete_pop(%d): m_readIndex = %d\n", idx, m_readIndex);

                readValue = m_buffer[m_readIndex];
                m_readIndex = (m_readIndex + 1) & (ARRAYSIZE - 1);
                --m_occupiedCells;

                om_async_operation_t->set_result_and_complete(readValue);
            }
            else
            {
                print(PRI1, "async_queue<...>::complete_pop(idx = %d): om_async_operation_t = nullptr\n", idx);
            }
        }

    private:
        std::array<TYPE, ARRAYSIZE> m_buffer;
        size_t m_occupiedCells{ 0 };
        int m_writeIndex{ 0 };
        int m_readIndex{ 0 };

        TYPE m_value_to_be_pushed{};
        int m_push_operation_index{ -1 };
        int m_pop_operation_index{ -1 };
    };
}

#endif
