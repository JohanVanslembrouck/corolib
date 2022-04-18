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

#include <coroutine>
#include "print.h"
#include "commservice.h"
#include "wait_all_counter.h"
#include "wait_any_one.h"

namespace corolib
{
    class async_operation_base
    {
    public:
        async_operation_base(CommService* s = nullptr, int index = 0);
        virtual ~async_operation_base();
		
        async_operation_base(const async_operation_base& s) = delete;
        async_operation_base(async_operation_base&& s) noexcept;

        async_operation_base& operator = (const async_operation_base&) = delete;
        async_operation_base& operator = (async_operation_base&& s) noexcept;
    
        /**
         * @brief completed is called from the callback function
         * on completion of the asynchronous operation.
         * Implemented in async_operation.cpp.
         * This function should be called after the call of async_operation<TYPE>::set_result(TYPE).
         */
        void completed();
		
        void setCounter(wait_all_counter* ctr)
        {
            print(PRI2, "%p: void async_operation_base::setCounter(%p)\n", this, ctr);
            m_ctr = ctr;
        }

        void setWaitAny(wait_any_one* waitany)
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
        std::coroutine_handle<> m_awaiting;
        bool m_ready;
        bool m_autoreset;
        int m_index;
        wait_all_counter* m_ctr;
        wait_any_one* m_waitany;
    };

    template<typename TYPE>
    class async_operation : public async_operation_base
    {
    private:
        TYPE m_result;

    public:
        /**
         * @brief async_operation is the constructor for an asynchronous operation.
         * @param s
         * @param index
         */
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
        /**
         * @brief set_result sets the value of m_result.
         * This function has to be called from the callback function
         * with the result of the asynchronous operation.
         * (The callback function is typically called from a communication framework.)
         * @param result
         */
        void set_result(TYPE result)
        {
            print(PRI2, "%p: async_operation<TYPE>::set_result(...)\n", this);
            m_result = result;
        }

        /**
         * @brief get_result allows retrieving the result of the asynchronous operation.
         * It can be used as an alternative to co_await and after co_await was called:
         * the operation has to be completed because otherwise the value of m_result
         * ïs still the one initialized in the constructor.
         * The function is usually called after using co_await wait_all or co_await wait_any,
         * because these objects cannot return the result of one or all of
         * their contained asynchronous operations.
         * @return
         */
        TYPE get_result()
        {
            print(PRI2, "%p: async_operation<TYPE>::get_result()\n", this);
            return m_result;
        }

        /**
         * @brief operator co_await allows to use async_operation as an awaitable.
         */
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

                void await_suspend(std::coroutine_handle<> awaiting)
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

                void await_suspend(std::coroutine_handle<> awaiting)
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
