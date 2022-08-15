/**
 * @file when_any.h
 * @brief
 * when_any waits for any of the async_operation or async_task objects passed to it in its constructor
 * to complete. These objects are placed in a vector m_elements.
 * 
 * For every async_operation or async_task object in m_elements, when_any creates a wait_any_on
 * object and places it in a second vector m_wait_any.
 * It associates each element in m_elements with the corresponding element in m_wait_any.
 * This way the async_operation or async_task object in m_elements can
 * inform when_any via the corresponding element in m_wait_any that it has completed.
 *
 * TODO1: verify instantiation of when_any with an appropriate type using C++20 concepts.
 * TODO2: implement other ways to pass the async_operation or async_task objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ANY_AWAITABLE_
#define _WHEN_ANY_AWAITABLE_

#include <vector>

#include "print.h"
#include "when_any_one.h"
#include "async_operation.h"

namespace corolib
{
    template<typename TYPE>
    class when_any
    {
    public:
	    /**
         * @brief constructor that takes an initializer list and
		 * populates the internal vector m_elements with its elements.
         */
        when_any(std::initializer_list<TYPE*> aws)
        {
            print(PRI2, "%p: when_any::when_any(std::initializer_list<TYPE*> aws)\n", this);
            int i = 0;
            for (TYPE* a : aws)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!a->is_ready())
                {
                    when_any_one* q = new when_any_one();
                    m_wait_any.push_back(q);
                    a->setWaitAny(q);
                    m_elements.push_back(a);
                }
            }
        }

        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
		 * and that populates the internal vector m_elements with its elements.
         */
        when_any(TYPE* aws, int size)
        {
            print(PRI2, "%p: when_any::when_any(TYPE* aws, int size)\n", this);
            for (int i = 0; i < size; i++)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!aws[i].is_ready())
                {
                    when_any_one* q = new when_any_one();
                    m_wait_any.push_back(q);
                    aws[i].setWaitAny(q);
                    m_elements.push_back(&aws[i]);
                }
            }
        }

        when_any(const when_any& s) = delete;

        when_any(when_any&& s)
        {
            print(PRI2, "%p: when_any::when_any(when_any&& s)\n", this);
        }

        ~when_any()
        {
            print(PRI2, "%p: when_any::~when_any()\n", this);
            for (int i = 0; i < m_wait_any.size(); i++)
            {
                m_elements[i]->setWaitAny(nullptr);
                delete m_wait_any[i];
            }
        }

        when_any& operator = (const when_any&) = delete;

        when_any& operator = (when_any&& s)
        {
            print(PRI2, "%p: when_any::when_any = (when_any&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_any& sync_) : m_sync(sync_) {}

                bool await_ready()
                {
                    print(PRI2, "%p: when_any::await_ready()\n", this);
                    for (int i = 0; i < m_sync.m_wait_any.size(); i++)
                    {
                        if (m_sync.m_wait_any[i]->get_completed())
                        {
                            print(PRI2, "%p: when_any::await_ready(): return true for i = %d;\n", this, i);
                            return true;
                        }
                    }
                    print(PRI2, "%p: when_any::await_ready(): return false;\n", this);
                    return false;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: when_any::await_suspend(...)\n", this);
                    for (auto el : m_sync.m_wait_any)
                    {
                        el->set_awaiting(awaiting);
                    }
                }

                int await_resume()
                {
                    // Find out which one has completed
                    print(PRI2, "%p: when_any::await_resume()\n", this);
                    for (int i = 0; i < m_sync.m_wait_any.size(); i++)
                    {
                        if (m_sync.m_wait_any[i]->get_and_reset_completed())
                        {
                            print(PRI2, "%p: when_any::await_resume(): return i = %d\n", i);
                            return i;
                        }
                    }
                    
                    print(PRI1, "%p: when_any::await_resume(): Error, none has completed, yet the coroutine has been resumed\n", this);
                    return -1;
                }
            private:
                when_any& m_sync;
            };

            return awaiter{ *this };
        }

    private:
        std::vector<when_any_one*> m_wait_any;
        std::vector<TYPE*> m_elements;
    };
}

#endif
