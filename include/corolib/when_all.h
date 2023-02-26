/**
 * @file when_all.h
 * @brief
 * when_all waits for all async_operation or async_task objects passed to it in its constructor
 * to complete.
 *
 * when_all passes its m_counter data member object to every async_operation or async_task object.
 * When an async_operation or async_task completes, it decrements the counter in the m_counter object.
 * When that counter reaches 0, the coroutines co_awaiting the when_all object will be resumed.
 *
 * TODO1: verify instantiation of when_all with an appropriate type using C++20 concepts.
 * TODO2: implement other ways to pass the async_operation or async_task objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ALL
#define _WHEN_ALL

#include <vector>

#include "print.h"
#include "when_all_counter.h"
#include "async_operation.h"

namespace corolib
{
    // TYPE must be an async_operation or an async_task
    template<typename TYPE>
    class when_all
    {
    public:
	    /**
         * @brief constructor that takes an initializer list and
		 * populates the internal vector m_elements with its elements.
         */
        when_all(std::initializer_list<TYPE*> async_ops)
            : m_counter(0)
        {
            print(PRI2, "%p: when_all::when_all(std::initializer_list<TYPE*> async_ops)\n", this);
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
        when_all(TYPE* async_ops, int size)
            : m_counter(0)
        {
            print(PRI2, "%p: when_all::when_all(TYPE* async_ops, int size)\n", this);
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

        when_all(const when_all& s) = delete;

        when_all(when_all&& s)
        {
            print(PRI2, "%p: when_all::when_all(when_all&& s)\n", this);
        }

        ~when_all()
        {
            print(PRI2, "%p: when_all::~when_all()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->setCounter(nullptr);
            }
        }

        when_all& operator = (const when_all&) = delete;

        when_all& operator = (when_all&& s)
        {
            print(PRI2, "%p: when_all::when_all = (when_all&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        /**
         * @brief
         *
         */
        void start_all()
        {
            print(PRI1, "%p: when_all:start_all()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->start();
            }
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_all& when_all_) 
                    : m_when_all(when_all_)
                {
                    print(PRI2, "%p: when_all::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    print(PRI2, "%p: when_all::awaiter::await_ready(): m_when_all.m_counter.get_counter() = %d;\n", 
                            this, m_when_all.m_counter.get_counter());
                    bool ready = (m_when_all.m_counter.get_counter() == 0);
                    print(PRI2, "%p: when_all::awaiter::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: when_all::awaiter::await_suspend(...)\n", this);
                    m_when_all.start_all();    // Will have no effect in case of an eager start
                    m_when_all.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    print(PRI2, "%p: when_all::awaiter::await_resume()\n", this);
                }
            private:
                when_all& m_when_all;
            };

            return awaiter{ *this };
        }

    private:
        when_all_counter m_counter;
        std::vector<TYPE*> m_elements;
    };
}

#endif
