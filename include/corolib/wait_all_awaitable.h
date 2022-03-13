/**
 * @file wait_all_awaiable.h
 * @brief
 * wait_all waits for all async_operation objects passed to it in its constructor
 * to complete.
 * wait_all passes the same wait_all_counter object to every async_operation.
 * When an async_operation completes, it decrements the counter in the wait_all_counter object.
 * When that counter reaches 0, the coroutines co_awaiting the wait_all object
 * will be resumed.
 * TODO1: verify instantiation of wait_all with an apppropriate type.
 * TODO2: implement other ways to pass the async_operation objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ALL_AWAITABLE
#define _WAIT_ALL_AWAITABLE

#include <vector>

#include "print.h"
#include "wait_all_counter.h"
#include "async_operation.h"

namespace corolib
{
    // TYPE must be an async_operation or an async_operation_t<TYPE2>
    template<typename TYPE>
    struct wait_all
    {
        wait_all(std::initializer_list<TYPE*> async_ops)
            : m_counter(async_ops.size())
        {
            print(PRI2, "%p: wait_all::wait_all(std::initializer_list<TYPE*> async_ops)\n", this);
            for (TYPE* async_op : async_ops)
            {
                async_op->setCounter(&m_counter);
                m_elements.push_back(async_op);
            }
        }

        wait_all(TYPE* async_ops, int size)
            : m_counter(size)
        {
            print(PRI2, "%p: wait_all::wait_all(TYPE* async_ops, int size)\n", this);
            for (int i = 0; i < size; i++)
            {
                async_ops[i].setCounter(&m_counter);
                m_elements.push_back(&async_ops[i]);
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
