/**
 * @file wait_any_awaitable.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ANY_AWAITABLE_
#define _WAIT_ANY_AWAITABLE_

#include <vector>

#include "print.h"
#include "wait_any.h"
#include "async_operation.h"

namespace corolib
{
    template<typename TYPE>
    struct wait_any
    {
        wait_any(std::initializer_list<TYPE*> aws)
        {
            print(PRI2, "%p: wait_any::wait_any(std::initializer_list<TYPE*> aws)\n", this);
            int i = 0;
            for (TYPE* a : aws)
            {
                wait_any_one* q = new wait_any_one();
                m_wait_any.push_back(q);
                a->setWaitAny(q);
                m_elements.push_back(a);
            }
        }

        wait_any(TYPE* aws, int size)
        {
            print(PRI2, "%p: wait_any::wait_any(TYPE* aws, int size)\n", this);
            for (int i = 0; i < size; i++)
            {
                wait_any_one* q = new wait_any_one();
                m_wait_any.push_back(q);
                aws[i].setWaitAny(q);
                m_elements.push_back(&aws[i]);
            }
        }

        wait_any(const wait_any& s) = delete;

        wait_any(wait_any&& s)
        {
            print(PRI2, "%p: wait_any::wait_any(wait_any&& s)\n", this);
        }

        ~wait_any()
        {
            print(PRI2, "%p: wait_any::~wait_any()\n", this);
            for (int i = 0; i < m_wait_any.size(); i++)
            {
                m_elements[i]->setWaitAny(nullptr);
                delete m_wait_any[i];
            }
        }

        wait_any& operator = (const wait_any&) = delete;

        wait_any& operator = (wait_any&& s)
        {
            print(PRI2, "%p: wait_any::wait_any = (wait_any&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(wait_any& sync_) : m_sync(sync_) {}

                bool await_ready()
                {
                    print(PRI2, "%p: wait_any::await_ready()\n", this);
                    for (int i = 0; i < m_sync.m_wait_any.size(); i++)
                    {
                        if (m_sync.m_wait_any[i]->get_completed())
                        {
                            print(PRI2, "%p: wait_any::await_ready(): return true for i = %d;\n", this, i);
                            return true;
                        }
                    }
                    print(PRI2, "%p: wait_any::await_ready(): return false;\n", this);
                    return false;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: wait_any::await_suspend(...)\n", this);
                    for (auto el : m_sync.m_wait_any)
                    {
                        el->set_awaiting(awaiting);
                    }
                }

                int await_resume()
                {
                    // Find out which one has completed
                    print(PRI2, "%p: wait_any::await_resume()\n", this);
                    for (int i = 0; i < m_sync.m_wait_any.size(); i++)
                    {
                        if (m_sync.m_wait_any[i]->get_and_reset_completed())
                        {
                            print(PRI2, "%p: wait_any::await_resume(): return i = %d\n", i);
                            return i;
                        }
                    }
                    
                    print(PRI1, "%p: wait_any::await_resume(): Error, none has completed, yet the coroutine has been resumed\n", this);
                    return -1;
                }
            private:
                wait_any& m_sync;
            };

            return awaiter{ *this };
        }

    private:
        std::vector<wait_any_one*> m_wait_any;
        std::vector<TYPE*> m_elements;
    };
}

#endif
