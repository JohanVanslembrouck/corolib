/**
 * @file async_operation.h
 * @brief
 * async_operation<TYPE> is used as the return types of asynchronous (I/O) operations 
 * (see commcore.h, commclient.h and commserver.h).
 * async_operation<TYPE> can be co_awaited upon.
 * The TYPE in async_operation<TYPE> corresponds to the real return type of the operation.
 * async_operation<void> should be used for operations that return void.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */
#ifndef _ASYNC_OPERATION_H_
#define _ASYNC_OPERATION_H_

#include <experimental/coroutine>
#include "print.h"
#include "commservice.h"
#include "wait_all_counter.h"
#include "wait_any.h"

namespace corolib
{
    class async_operation_base
    {
    public:
        async_operation_base(CommService* s = nullptr, int index = 0)
            : m_service(s)
            , m_awaiting(nullptr)            // initialized in await_suspend()
            , m_ready(false)                // set to true in completed()
            , m_autoreset(false)
            , m_index(index)
            , m_ctr(nullptr)
            , m_waitany(nullptr)
        {
            print(PRI2, "%p: async_operation_base::async_operation_base(CommService* s = %p, index = %d)\n", this, s, index);
            if (m_service)
            {
                if (m_service->m_async_operations[m_index] == nullptr)
                {
                    m_service->m_async_operations[m_index] = this;
                }
                else
                {
                    print(PRI1, "%p: async_operation_base::async_operation_base(): m_service->m_async_operations[%d] WRONGLY INITIALIZED!!!\n", this, m_index);
                }
            }
        }

        virtual ~async_operation_base()
        {
            print(PRI2, "%p: async_operation_base::~async_operation_base(): m_index = %d\n", this, m_index);
            m_ready = false;
            m_autoreset = false;
            if (m_index != -1)
            {
                if (m_service)
                {
                    m_service->m_async_operations[m_index] = nullptr;
                }
            }
            m_index = -1;
            m_ctr = nullptr;
            m_waitany = nullptr;
        }

        async_operation_base(const async_operation_base& s) = delete;

        async_operation_base(async_operation_base&& s) noexcept
            : m_service(s.m_service)
            , m_awaiting(s.m_awaiting)
            , m_ready(s.m_ready)
            , m_autoreset(s.m_autoreset)
            , m_index(s.m_index)
            , m_ctr(s.m_ctr)
            , m_waitany(s.m_waitany)
        {
            print(PRI2, "%p: async_operation_base::async_operation_base(async_operation_base&& s): s.m_index = %d\n", this, s.m_index);

            // Tell the CommService we are at another address after the move.
            m_service->m_async_operations[m_index] = this;

            s.m_service = nullptr;
            s.m_awaiting = nullptr;
            s.m_ready = false;
            s.m_autoreset = false;
            s.m_index = -1;        // indicates move
            s.m_ctr = nullptr;
            s.m_waitany = nullptr;
        }

        async_operation_base& operator = (const async_operation_base&) = delete;

        async_operation_base& operator = (async_operation_base&& s)
        {
            print(PRI2, "%p: async_operation_base::async_operation_base = (async_operation_base&& s): m_index = %d, s.m_index = %d\n", this, m_index, s.m_index);

            // Clean our entry at the original location, because we will move to another one.
            if (m_service)
                m_service->m_async_operations[m_index] = nullptr;

            m_service = s.m_service;
            m_awaiting = s.m_awaiting;
            m_ready = s.m_ready;
            m_autoreset = s.m_autoreset;
            m_index = s.m_index;

            // The following 2 tests allow an async_operation that takes part in
            // a wait_all_awaitable or wait_any_awaitable to be re-assigned.
            // This avoids disposing the original wait_all_awaitable or wait_any_awaitable
            // and constructing a new one.
            if (m_ctr != nullptr && s.m_ctr == nullptr)
                ; // do not overwrite m_ctr
            else
                m_ctr = s.m_ctr;
            if (m_waitany != nullptr && s.m_waitany == nullptr)
                ; // do not overwrite m_waitany
            else
                m_waitany = s.m_waitany;

            // Tell the CommService we are at another address after the move.
            if (m_service)
                m_service->m_async_operations[m_index] = this;

            s.m_service = nullptr;
            s.m_awaiting = nullptr;
            s.m_ready = false;
            s.m_autoreset = false;
            s.m_index = -1;        // indicates move
            s.m_ctr = nullptr;
            s.m_waitany = nullptr;
            
            return *this;
        }

        void completed()
        {
            print(PRI2, "%p: async_operation_base::completed()\n", this);
            if (m_awaiting)
            {
                print(PRI2, "%p: async_operation_base::completed(): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                m_ready = true;
                print(PRI2, "%p: async_operation_base::completed(): after m_awaiting.resume();\n", this);
            }
            else if (m_ctr)
            {
                print(PRI2, "%p: async_operation_base::completed(): before m_ctr->completed();\n", this);
                m_ctr->completed();
                print(PRI2, "%p: async_operation_base::completed(): after m_ctr->completed();\n", this);
            }
            else if (m_waitany)
            {
                print(PRI2, "%p: async_operation_base::completed(): before m_waitany->completed();\n", this);
                m_waitany->completed();
                print(PRI2, "%p: async_operation_base::completed(): after m_waitany->completed();\n", this);
            }
            else
            {
                print(PRI2, "%p: async_operation_base::completed(): m_awaiting not yet initialized!!!\n", this);
                print(PRI2, "%p: async_operation_base::completed(): operation completed before co_waited!!\n", this);
                m_ready = true;     // Set to completed.
            }
        }

        void setCounter(wait_all_counter* ctr)
        {
            print(PRI2, "%p: void async_operation_base::setCounter(%p)\n", this, ctr);
            m_ctr = ctr;
        }

        void setWaitAny(wait_any* waitany)
        {
            print(PRI2, "%p: void async_operation_base::setWaitAny(%p)\n", this, waitany);
            m_waitany = waitany;
        }

        int get_index()
        {
            return m_index;
        }

        /**
         * @brief reset Allows the same operation to be co_awaited again
         * without returning immediately.
         */
        void reset()
        {
            m_ready = false;
            m_awaiting = nullptr;
        }

        void auto_reset(bool autoreset)
        {
            m_autoreset = autoreset;
        }

    protected:
        CommService* m_service;
        std::experimental::coroutine_handle<> m_awaiting;
        bool m_ready;
        bool m_autoreset;
        int m_index;
        wait_all_counter* m_ctr;
        wait_any* m_waitany;
    };

    template<typename TYPE>
    class async_operation : public async_operation_base
    {
    private:
        TYPE m_result;

    public:
        async_operation(CommService* s = nullptr, int index = 0)
            : async_operation_base(s, index)
            , m_result{}
        {
            print(PRI2, "%p: async_operation<TYPE>::async_operation()\n", this);
        }
#if 0
        void set_result(std::string result)
        {
            print(PRI2, "%p: async_operation<TYPE>::set_result(...): result = %s\n", this, result.c_str());
            m_result = result;
        }
#endif
        void set_result(TYPE result)
        {
            print(PRI2, "%p: async_operation<TYPE>::set_result(...)\n", this);
            m_result = result;
        }

        TYPE get_result()
        {
            print(PRI2, "%p: async_operation<TYPE>::get_result()\n", this);
            return m_result;
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(async_operation& async_) :
                    m_async(async_)
                {}

                bool await_ready()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::await_ready(): return %d;\n", this, &m_async, m_async.m_ready);
                    return m_async.m_ready;
                }

                void await_suspend(std::experimental::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::await_suspend(...)\n", this, &m_async);
                    m_async.m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::await_resume()\n", this, &m_async);
                    if (m_async.m_autoreset)
                        m_async.reset();
                    return m_async.m_result;
                }

            private:
                async_operation& m_async;
            };

            return awaiter{ *this };
        }

    };

    template<>
    class async_operation<void> : public async_operation_base
    {
    public:
        async_operation(CommService* s = nullptr, int index = 0)
            : async_operation_base(s, index)
        {
            print(PRI2, "%p: async_operation<void>::async_operation()\n", this);
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(async_operation<void>& async_)
                    : m_async(async_)
                {}

                bool await_ready()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_ready(): return %d;\n", this, &m_async, m_async.m_ready);
                    return m_async.m_ready;
                }

                void await_suspend(std::experimental::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_suspend(...)\n", this, &m_async);
                    m_async.m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_resume()\n", this, &m_async);
                    if (m_async.m_autoreset)
                        m_async.reset();
                }

            private:
                async_operation<void>& m_async;
            };

            return awaiter{ *this };
        }
    };

}

#endif
