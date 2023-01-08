/**
 * @file async_task.h
 * @brief
 * async_task<TYPE> defines an eager awaitable type: its promise_type::initial_suspend function returns std::suspend_never{}.
 * async_ltask<TYPE> defines a lazy awaitable type: its promise_type::initial_suspend function returns std::suspend_always{}.
 * 
 * Both types include a promise_type, so they can be used as the return type of a coroutine.
 * 
 * They also define operator co_await, so another coroutine can co_await the async_task<TYPE> or async_ltask<TYPE> object.
 * The TYPE in async_task<TYPE> or async_ltask<TYPE> corresponds to the "real" return type of the coroutine
 * (the type the user is interested in).
 * 
 * async_task<TYPE> and async_ltask<TYPE> are derived from a common base class async_task_base<TYPE>.
 * 
 * Note: there is currently no implementation of async_ltask<void>.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _ASYNC_TASK_H_
#define _ASYNC_TASK_H_

#include <coroutine>
#include "print.h"
#include "semaphore.h"
#include "when_all_counter.h"
#include "when_any_one.h"

namespace corolib
{
    template<typename TYPE>
    class async_task_base
    {
    public:
	
        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        async_task_base(const async_task_base& s) = delete;

        async_task_base(async_task_base&& s)
            : m_coro(s.m_coro)
        {
            print(PRI2, "%p: async_task_base::async_task_base(async_task_base&& s)\n", this);
            s.m_coro = nullptr;
        }

        ~async_task_base()
        {
            print(PRI2, "%p: async_task_base::~async_task_base()\n", this);
        }

        async_task_base(handle_type h)
            : m_coro(h) {
            print(PRI2, "%p: async_task_base::async_task_base(handle_type h)\n", this);
        }

        async_task_base& operator = (const async_task_base&) = delete;

        async_task_base& operator = (async_task_base&& s)
        {
            print(PRI2, "%p: async_task_base::async_task_base = (async_task_base&& s)\n", this);
            m_coro = s.m_coro;
            s.m_coro = nullptr;
            return *this;
        }

        /**
         * @brief get_result retrieves the result of the embedded promise.
         * If the coroutine returning the async_task_base object has not yet returned,
         * get_result will wait for the semaphone to be signaled,
         * which can only be done from another thread
         * (the wait() call on the semaphore will block the current thread).
         * Therefore:
         * Use co_await first to make sure that the coroutine has returned.
         * If this is not possible (because you cannot use co_await in a normal function),
         * make sure there is another thread that will
         * resume the coroutine and make it return.
         * @return
         */
        TYPE get_result()
        {
            print(PRI2, "%p: async_task_base::get_result()\n", this);
            if (!m_coro.promise().m_ready)
            {
                m_coro.promise().m_wait_for_signal = true;
                m_coro.promise().m_sema.wait();
            }
            return m_coro.promise().m_value;
        }

        bool is_ready()
        {
            return m_coro.promise().m_ready;
        }

        /**
         * @brief called from the constructors and destructor of when_all
         *
         */
        void setCounter(when_all_counter* ctr)
        {
            print(PRI2, "%p: void m_async_task_base::setCounter(%p)\n", this, ctr);
            m_coro.promise().m_ctr = ctr;
        }

        /**
		 * @brief called from the constructors and destructor of when_any
         *
         */
        void setWaitAny(when_any_one* waitany)
        {
            print(PRI2, "%p: void m_async_task_base::setWaitAny(%p)\n", this, waitany);
            m_coro.promise().m_waitany = waitany;
        }

        struct promise_type
        {
            friend class async_task_base;

            promise_type()
                : m_awaiting(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
                , m_value{}
                , m_ready(false)
                , m_wait_for_signal(false)
            {
                print(PRI2, "%p: async_task_base::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                print(PRI2, "%p: async_task_base::promise_type::~promise_type()\n", this);
            }


            void return_value(TYPE v)
            {
                print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): begin\n", this);
                m_value = v;
                m_ready = true;

                if (m_ctr)
                {
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): before m_ctr->completed();\n", this);
                    m_ctr->completed();
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): before m_waitany->completed();\n", this);
                    m_waitany->completed();
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): after m_waitany->completed();\n", this);
                    return;
                }

                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): after m_awaiting.resume();\n", this);
                }
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): before sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): after sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task::promise_type::return_value(TYPE v): end\n", this);
            }

            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task_base::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }

            void unhandled_exception()
            {
                print(PRI2, "%p: async_task_base::promise::promise_type()\n", this);
                std::exit(1);
            }

        public:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            TYPE m_value;
            bool m_ready;
            bool m_wait_for_signal;
        };

    protected:
        handle_type m_coro;
    };


    template<typename TYPE>
    class async_task : public async_task_base<TYPE>
    {
    public:
        async_task(handle_type h)
            : async_task_base(h) {
            print(PRI2, "%p: async_task::async_task(handle_type h)\n", this);
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(async_task& async_task_)
                    : m_async_task(async_task_)
                {}

                bool await_ready()
                {
                    const bool ready = m_async_task.m_coro.done();
                    print(PRI2, "%p: m_async_task::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async_task::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async_task::await_resume()\n", this);
                    const TYPE r = m_async_task.m_coro.promise().m_value;
                    return r;
                }

            private:
                async_task& m_async_task;
            };

            return awaiter{ *this };
        }

        struct promise_type : public async_task_base<TYPE>::promise_type
        {
            friend class async_task;
            
            auto get_return_object()
            {
                print(PRI2, "%p: async_task::promise_type::get_return_object()\n", this);
                return async_task<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }
        };

    };

    template<typename TYPE>
    class async_ltask : public async_task_base<TYPE>
    {
    public:
        async_ltask(handle_type h)
            : async_task_base(h) {
            print(PRI2, "%p: async_ltask::async_task(handle_type h)\n", this);
        }

        void start()
        {
            m_coro.resume();
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(async_ltask& async_ltask_)
                    : m_async_ltask(async_ltask_)
                {}

                bool await_ready()
                {
                    const bool ready = m_async_ltask.m_coro.done();
                    print(PRI2, "%p: m_async_ltask::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    m_async_ltask.m_coro.resume();
                    print(PRI2, "%p: m_async_ltask::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_ltask.m_coro.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async_ltask::await_resume()\n", this);
                    const TYPE r = m_async_ltask.m_coro.promise().m_value;
                    return r;
                }

            private:
                async_ltask& m_async_ltask;
            };

            return awaiter{ *this };
        }

        struct promise_type : public async_task_base<TYPE>::promise_type
        {
            friend class async_ltask;
 
            auto get_return_object()
            {
                print(PRI2, "%p: async_ltask::promise_type::get_return_object()\n", this);
                return async_ltask<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }
        };

    };


    template<>
    class async_task<void>
    {
    public:

        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        async_task(const async_task& s) = delete;

        async_task(async_task&& s) noexcept
            : m_coro(s.m_coro)
        {
            print(PRI2, "%p: async_task::async_task(async_task&& s)\n", this);
            s.m_coro = nullptr;
        }

        ~async_task()
        {
            print(PRI2, "%p: async_task::~async_task()\n", this);
        }

        async_task(handle_type h)
            : m_coro(h) {
            print(PRI2, "%p: async_task::async_task(handle_type h)\n", this);
        }

        async_task& operator = (const async_task&) = delete;

        async_task& operator = (async_task&& s) noexcept
        {
            print(PRI2, "%p: async_task::async_task = (async_task&& s)\n", this);
            m_coro = s.m_coro;
            s.m_coro = nullptr;
            return *this;
        }

        /**
         * @brief wait is the counterpart of get_result for an asynchronous task
         * that returns void instead of any other type.
         * See get_result for more information on its use.
         */
        void wait()
        {
            print(PRI2, "%p: async_task::wait()\n", this);
            if (!m_coro.promise().m_ready)
            {
                m_coro.promise().m_wait_for_signal = true;
                m_coro.promise().m_sema.wait();
            }
        }

        bool is_ready()
        {
            return m_coro.promise().m_ready;
        }

        void setCounter(when_all_counter* ctr)
        {
            print(PRI2, "%p: void m_async_task::setCounter(%p)\n", this, ctr);
            m_coro.promise().m_ctr = ctr;
        }

        void setWaitAny(when_any_one* waitany)
        {
            print(PRI2, "%p: void m_async_task::setWaitAny(%p)\n", this, waitany);
            m_coro.promise().m_waitany = waitany;
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(async_task& async_task_)
                    : m_async_task(async_task_)
                {}

                bool await_ready()
                {
                    const bool ready = m_async_task.m_coro.done();
                    print(PRI2, "%p: m_async_task::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: m_async_task::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro.promise().m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: m_async_task::await_resume()\n", this);
                }

            private:
                async_task& m_async_task;
            };

            return awaiter{ *this };
        }

        struct promise_type
        {
            friend class async_task;

            promise_type()
                : m_awaiting(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
                , m_ready(false)
                , m_wait_for_signal(false)
            {
                print(PRI2, "%p: async_task::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                print(PRI2, "%p: async_task::promise_type::~promise_type()\n", this);
            }

            void return_void()
            {
                print(PRI2, "%p: async_task::promise_type::return_void(): begin\n", this);
                m_ready = true;

                if (m_ctr)
                {
                    print(PRI2, "%p: async_task::promise_type::return_void(): before m_ctr->completed();\n", this);
                    m_ctr->completed();
                    print(PRI2, "%p: async_task::promise_type::return_void(): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task::promise_type::return_void(): before m_waitany->completed();\n", this);
                    m_waitany->completed();
                    print(PRI2, "%p: async_task::promise_type::return_void(): after m_waitany->completed();\n", this);
                    return;
                }

                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task::promise_type::return_void(): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task::promise_type::return_void(): after m_awaiting.resume();\n", this);
                }
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task::promise_type::return_void(): before sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task::promise_type::return_void(): after sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task::promise_type::return_void(): end\n", this);
            }

            auto get_return_object()
            {
                print(PRI2, "%p: async_task::promise_type::get_return_object()\n", this);
                return async_task<void>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }

            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }

            void unhandled_exception()
            {
                print(PRI2, "%p: async_task::promise::promise_type()\n", this);
                std::exit(1);
            }

        private:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            bool m_ready;
            bool m_wait_for_signal;
        };

    private:
        handle_type m_coro;
    };

}

#endif
