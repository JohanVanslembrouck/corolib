/**
 * @file async_operation.h
 * @brief
 * async_operation<TYPE> is used as the return types of asynchronous (I/O) operations.
 * async_operation<TYPE> defines operator co_await (so it can be co_awaited upon from a coroutine)
 * but it does not define a promise_type (so it can not be used as a return type of a coroutine).
 *
 * The TYPE in async_operation<TYPE> corresponds to the real return type of the operation.
 * async_operation<void> must be used for operations that return void.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

 /**
 Class hierarchy
 ===============

                     class async_base
                             ^
                             |
                             |
                class async_operation_base
                             ^
                             |
                             |
             ----------------------------------
             |                                |
             |                                |
     template<typename TYPE>            template<>
     class async_operation              class async_operation<void>

*/

#ifndef _ASYNC_OPERATION_H_
#define _ASYNC_OPERATION_H_

#include <vector>           // RESUME_MULTIPLE_COROUTINES

#include <system_error>
#include <coroutine>

#include "print.h"
#include "tracker.h"
#include "async_base.h"
#include "when_all_counter.h"
#include "when_any_one.h"

namespace corolib
{
    class CommService;
  
    void resume_multiple_coroutines(bool allow = false);
    extern bool _resume_multiple_coroutines;

    class async_operation_base : public async_base, operation_tracker
    {
    public:
        async_operation_base(CommService* s = nullptr, int index = -1, bool timestamp = false);
        virtual ~async_operation_base();
        
        async_operation_base(const async_operation_base&) = delete;
        async_operation_base(async_operation_base&&) noexcept;

        async_operation_base& operator = (const async_operation_base&) = delete;
        async_operation_base& operator = (async_operation_base&&) noexcept;
    
        /**
         * @brief completed is called from the callback function
         * on completion of the asynchronous operation.
         * Implemented in async_operation.cpp.
         * This function should be called after the call of async_operation<TYPE>::set_result(TYPE).
         */
        void completed();
        
        bool is_ready() override
        {
#if USE_IN_MT_APPS
            print(PRI2, "%p: void async_operation_base::is_ready() returns %d\n", this, m_ready.load());
#else
            print(PRI2, "%p: void async_operation_base::is_ready() returns %d\n", this, m_ready);
#endif
            return m_ready;
        }

        /**
         * @brief called from the constructors and destructor of when_all
         *
         */
        void setCounter(when_all_counter* ctr) override
        {
            print(PRI2, "%p: void async_operation_base::setCounter(%p)\n", this, ctr);
            m_ctr = ctr;
        }

        /**
         * @brief called from the constructors and destructor of when_any
         *
         */
        void setWaitAny(when_any_one* waitany) override
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

            if (corolib::_resume_multiple_coroutines)
                ;  //m_awaitings.clear();      // make empty??? Should only do it for a single coroutine
            else
                m_awaiting = nullptr;
        }

        void auto_reset(bool autoreset)
        {
            m_autoreset = autoreset;
        }

        /**
         * @brief start is a dummy function for asynchronous operations, but it required because
         * of its call in when_all and when_any.
         */
        void start() override
        {
        }

    protected:
        void inform_interested_parties();
        void cleanup();

    protected:
        CommService* m_service;
        std::vector<std::coroutine_handle<>> m_awaitings;       // RESUME_MULTIPLE_COROUTINES
        std::coroutine_handle<> m_awaiting;                     // !RESUME_MULTIPLE_COROUTINES
        when_all_counter* m_ctr;
        when_any_one* m_waitany;
        int m_index;
#if USE_IN_MT_APPS
        std::atomic<bool> m_ready;
#else
        bool m_ready;
#endif
        bool m_autoreset;
        bool m_timestamp;
    }; // class async_operation_base


    template<typename TYPE>
    class async_operation : public async_operation_base
    {
    public:
        /**
         * @brief async_operation is the constructor for an asynchronous operation.
         * @param s
         * @param index
         */
        async_operation(CommService* s = nullptr, int index = -1, bool timestamp = false)
            : async_operation_base(s, index, timestamp)
            , m_result{}
            , m_errorCode{ 0 }
        {
            print(PRI2, "%p: async_operation<TYPE>::async_operation()\n", this);
        }

        ~async_operation()
        {
            print(PRI2, "%p: async_operation<TYPE>::~async_operation()\n", this);
        }

        async_operation(const async_operation& other) = delete;
        async_operation(async_operation&& other) noexcept
            : async_operation_base(std::move(other))
            , m_result(std::move(other.m_result))
            , m_errorCode(other.m_errorCode)
        {
            print(PRI2, "%p: async_operation<TYPE>::async_operation((async_operation&& s)\n", this);
        }

        async_operation& operator = (const async_operation&) = delete;
        async_operation& operator = (async_operation&& other) noexcept
        {
            print(PRI2, "%p: async_operation<TYPE>::operator = (async_operation&& s)\n", this);
            async_operation_base::operator = (std::move(other));
            m_result = std::move(other.m_result);
            m_errorCode = other.m_errorCode;
            return *this;
        }

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

        void set_error(int errorCode)
        {
            print(PRI1, "%p: async_operation<TYPE>::set_error(%d)\n", this, errorCode);
            m_errorCode = errorCode;
        }

        /**
         * @brief set_result_and_complete combines set_result and completed
         */
        void set_result_and_complete(TYPE result)
        {
            print(PRI2, "%p: async_operation<TYPE>::set_result_and_complete(...)\n", this);
            set_result(result);
            completed();
        }

        /**
         * @brief get_result allows retrieving the result of the asynchronous operation.
         * It can be used as an alternative to co_await and after co_await was called:
         * the operation has to be completed because otherwise the value of m_result
         * ïs still the one initialized in the constructor.
         * The function is usually called after using co_await when_all or co_await when_any,
         * because these objects cannot return the result of one or all of
         * their contained asynchronous operations.
         * @return
         */
        TYPE get_result()
        {
            print(PRI2, "%p: async_operation<TYPE>::get_result()\n", this);
            if (m_errorCode != 0)
            {
                print(PRI1, "%p: async_operation<TYPE>::get_result(): m_errorCode = %d: throw exception!\n", this, m_errorCode);
                throw std::system_error{ m_errorCode, std::system_category() };
            }
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
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::awaiter::awaiter()\n", this, &m_async);
                }

                bool await_ready()
                {
#if USE_IN_MT_APPS
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::awaiter::await_ready(): return %d;\n", this, &m_async, m_async.m_ready.load());
#else
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::awaiter::await_ready(): return %d;\n", this, &m_async, m_async.m_ready);
#endif
                    return m_async.m_ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::awaiter::await_suspend(...)\n", this, &m_async);
                    if (corolib::_resume_multiple_coroutines)
                        m_async.m_awaitings.push_back(awaiting);
                    else
                        m_async.m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<TYPE>::await_resume()\n", this, &m_async);
                    // To be placed in void async_operation_base::completed(); // ???
                    if (m_async.m_autoreset)
                        m_async.reset();
                    return m_async.get_result();
                }

            private:
                async_operation& m_async;
            };

            return awaiter{ *this };
        }

    private:
        TYPE m_result;
        int m_errorCode;
    }; // template<typename TYPE> class async_operation


    template<>
    class async_operation<void> : public async_operation_base
    {
    public:
        async_operation(CommService* s = nullptr, int index = -1, bool timestamp = false)
            : async_operation_base(s, index, timestamp)
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
#if USE_IN_MT_APPS
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_ready(): return %d;\n", this, &m_async, m_async.m_ready.load());
#else
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_ready(): return %d;\n", this, &m_async, m_async.m_ready);
#endif
                    return m_async.m_ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_suspend(...)\n", this, &m_async);
                    if (corolib::_resume_multiple_coroutines)
                        m_async.m_awaitings.push_back(awaiting);
                    else
                        m_async.m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: m_async = %p: async_operation<void>::await_resume()\n", this, &m_async);
                    // To be placed in void async_operation_base::completed(); // ???
                    if (m_async.m_autoreset)
                        m_async.reset();
                }

            private:
                async_operation<void>& m_async;
            };

            return awaiter{ *this };
        }
    }; // template<> class async_operation<void>

} // namespace corolib

#endif
