/**
 * @file wait_all_awaitable.h
 * @brief
 * wait_all waits for all async_operation or async_task objects passed to it in its constructor
 * to complete.
 *
 * wait_all passes its m_counter data member object to every async_operation or async_task object.
 * When an async_operation or async_task completes, it decrements the counter in the m_counter object.
 * When that counter reaches 0, the coroutines co_awaiting the wait_all object will be resumed.
 *
 * TODO1: verify instantiation of wait_all with an appropriate type using C++20 concepts.
 * TODO2: implement other ways to pass the async_operation or async_task objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ALL_AWAITABLE
#define _WAIT_ALL_AWAITABLE

#include <vector>

#include "print.h"
#include "wait_all_counter.h"
#include "async_operation.h"

namespace corolib
{
    // TYPE must be an async_operation or an async_task
    template<typename TYPE>
    class wait_all
    {
    public:
	    /**
         * @brief constructor that takes an initializer list and
		 * populates the internal vector m_elements with its elements.
         */
        wait_all(std::initializer_list<TYPE*> async_ops)
            : m_counter(0)
        {
            print(PRI2, "%p: wait_all::wait_all(std::initializer_list<TYPE*> async_ops)\n", this);
            for (TYPE* async_op : async_ops)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!async_op->is_ready())
                {
                    async_op->setCounter(&m_counter);
                    m_elements.push_back(async_op);
                    m_counter.increment();
                }
            }
        }

        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
		 * and that populates the internal vector m_elements with its elements.
         */
        wait_all(TYPE* async_ops, int size)
            : m_counter(0)
        {
            print(PRI2, "%p: wait_all::wait_all(TYPE* async_ops, int size)\n", this);
            for (int i = 0; i < size; i++)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!async_ops[i].is_ready())
                {
                    async_ops[i].setCounter(&m_counter);
                    m_elements.push_back(&async_ops[i]);
                    m_counter.increment();
                }
            }
        }

        wait_all(const wait_all& s) = delete;

        wait_all(wait_all&& s)
        {
            print(PRI2, "%p: wait_all::wait_all(wait_all&& s)\n", this);
        }

        ~wait_all()
        {
            print(PRI2, "%p: wait_all::~wait_all()\n", this);
            for (int i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->setCounter(nullptr);
            }
        }

        wait_all& operator = (const wait_all&) = delete;

        wait_all& operator = (wait_all&& s)
        {
            print(PRI2, "%p: wait_all::wait_all = (wait_all&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(wait_all& sync_) : m_sync(sync_) {}

                bool await_ready()
                {
                    print(PRI2, "%p: wait_all::await_ready(): m_sync.m_counter.get_counter() = %d;\n", this, m_sync.m_counter.get_counter());
                    bool ready = (m_sync.m_counter.get_counter() == 0);
                    print(PRI2, "%p: wait_all::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: wait_all::await_suspend(...)\n", this);
                    m_sync.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    print(PRI2, "%p: wait_all::await_resume()\n", this);
                }
            private:
                wait_all& m_sync;
            };

            return awaiter{ *this };
        }

    private:
        wait_all_counter m_counter;
        std::vector<TYPE*> m_elements;
    };
}

#endif
