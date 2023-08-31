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
 * async_task<void> and async_ltask<void> are derived from a common base class async_task_void.
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
            : m_coro(h)
        {
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
         * @brief Starts a lazy coroutine.
         * Should be a member function of async_ltask, but g++ does not find m_coro.
         */
        void baseStart()
        {
            m_coro.resume();
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
         * @brief waitIfNotReady instructs get_result to wait by default
         * if the promise is not ready.
         * @return
         */
        TYPE get_result(bool waitIfNotReady = true)
        {
            print(PRI2, "%p: async_task_base::get_result()\n", this);
            if (!m_coro.promise().m_ready)
            {
                if (waitIfNotReady)
                {
                    print(PRI2, "%p: async_task_base::get_result(): waiting for signal\n", this);
                    m_coro.promise().m_wait_for_signal = true;
                    m_coro.promise().m_sema.wait();
                }
                else
                {
                    print(PRI1, "%p: async_task_base::get_result(): returning NULL value!\n", this);
                    return {};
                }
            }
            print(PRI2, "%p: async_task_base::get_result(): return m_coro.promise().m_value;\n", this);
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
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): begin\n", this);
                m_value = v;
                m_ready = true;

                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_ctr->completed();\n", this);
                    m_ctr->completed();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_waitany->completed();\n", this);
                    m_waitany->completed();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_waitany->completed();\n", this);
                    return;
                }
                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_awaiting.resume();\n", this);
                }
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): end\n", this);
            }

            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task_base::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }

            void unhandled_exception()
            {
                print(PRI2, "%p: async_task_base::promise_type::unhandled_exception()\n", this);
                m_exception = std::current_exception();
            }

            TYPE get_result()
            {
                print(PRI2, "%p: async_task_base::promise_type::get_result()\n", this);
                if (m_exception != nullptr)
                {
                    print(PRI1, "%p: async_task_base::promise_type::get_result(): throwing exception\n", this);
                    std::rethrow_exception(m_exception);
                }
                return m_value;
            }

        public:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            TYPE m_value;
            bool m_ready;
            bool m_wait_for_signal;
        private:
            std::exception_ptr m_exception;
        };

    protected:
        handle_type m_coro;
    };


    template<typename TYPE>
    class async_task : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        async_task(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_task<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start is a dummy function for eager start coroutines, but it required because
         * of its call in when_all and when_any.
         */
        void start()
        {
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
                    print(PRI2, "%p: async_task<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: async_task<TYPE>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: async_task<TYPE>::await_resume()\n", this);
                    const TYPE r = m_async_task.m_coro.promise().get_result();
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
                print(PRI2, "%p: async_task<TYPE>::promise_type::get_return_object()\n", this);
                return async_task<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }
        };

    };

    template<typename TYPE>
    class async_ltask : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        async_ltask(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_ltask<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start starts a lazy coroutine.
         * 
         */
        void start()
        {
            this->baseStart();
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
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    m_async_ltask.m_coro.resume();
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_ltask.m_coro.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_resume()\n", this);
                    const TYPE r = m_async_ltask.m_coro.promise().get_result();
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
                print(PRI2, "%p: async_ltask<TYPE>::promise_type::get_return_object()\n", this);
                return async_ltask<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }
        };

    };


    class async_task_void
    {
    public:

        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        async_task_void(const async_task_void& s) = delete;

        async_task_void(async_task_void&& s) noexcept
            : m_coro(s.m_coro)
        {
            print(PRI2, "%p: async_task_void::async_task_void(async_task&& s)\n", this);
            s.m_coro = nullptr;
        }

        ~async_task_void()
        {
            print(PRI2, "%p: async_task_void::~async_task_void()\n", this);
        }

        async_task_void(handle_type h)
            : m_coro(h)
        {
            print(PRI2, "%p: async_task_void::async_task_void(handle_type h)\n", this);
        }

        async_task_void& operator = (const async_task_void&) = delete;

        async_task_void& operator = (async_task_void&& s) noexcept
        {
            print(PRI2, "%p: async_task_void::async_task_void = (async_task&& s)\n", this);
            m_coro = s.m_coro;
            s.m_coro = nullptr;
            return *this;
        }

        /**
         * @brief baseStart() starts a lazy coroutine.
         * Should be a member function of async_ltask, but g++ does not find m_coro.
         */
        void baseStart()
        {
            m_coro.resume();
        }

        /**
         * @brief wait is the counterpart of get_result for an asynchronous task
         * that returns void instead of any other type.
         * See get_result for more information on its use.
         * @brief waitIfNotReady instructs get_result to wait by default
         * if the promise is not ready.
         */
        void wait(bool waitIfNotReady = true)
        {
            print(PRI2, "%p: async_task_void::wait()\n", this);
            if (!m_coro.promise().m_ready)
            {
                if (waitIfNotReady)
                {
                    m_coro.promise().m_wait_for_signal = true;
                    m_coro.promise().m_sema.wait();
                }
                else
                {
                    print(PRI1, "%p: async_task_void::wait(): returning without ready\n", this);
                }
            }
        }

        bool is_ready()
        {
            return m_coro.promise().m_ready;
        }

        void setCounter(when_all_counter* ctr)
        {
            print(PRI2, "%p: void async_task_void::setCounter(%p)\n", this, ctr);
            m_coro.promise().m_ctr = ctr;
        }

        void setWaitAny(when_any_one* waitany)
        {
            print(PRI2, "%p: void async_task_void::setWaitAny(%p)\n", this, waitany);
            m_coro.promise().m_waitany = waitany;
        }

        struct promise_type
        {
            friend class async_task_void;

            promise_type()
                : m_awaiting(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
                , m_ready(false)
                , m_wait_for_signal(false)
            {
                print(PRI2, "%p: async_task_void::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                print(PRI2, "%p: async_task_void::promise_type::~promise_type()\n", this);
            }

            void return_void()
            {
                print(PRI2, "%p: async_task_void::promise_type::return_void(): begin\n", this);
                m_ready = true;

                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_ctr->completed();\n", this);
                    m_ctr->completed();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_waitany->completed();\n", this);
                    m_waitany->completed();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_waitany->completed();\n", this);
                    return;
                }
                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_awaiting.resume();\n", this);
                }
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task_void::promise_type::return_void(): end\n", this);
            }

            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task_void::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }

            void unhandled_exception()
            {
                print(PRI2, "%p: async_task_void::promise_type::unhandled_exception()\n", this);
                m_exception = std::current_exception();
            }

            void get_result()
            {
                print(PRI2, "%p: async_task_void::promise_type::get_result()\n", this);
                if (m_exception != nullptr)
                {
                    print(PRI1, "%p: async_task_void::promise_type::get_result(): throwing exception\n", this);
                    std::rethrow_exception(m_exception);
                }
            }

        public:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            bool m_ready;
            bool m_wait_for_signal;
        private:
            std::exception_ptr m_exception;
        };

    protected:
        handle_type m_coro;
    };

    template<>
    class async_task<void> : public async_task_void
    {
    public:
	
	    using handle_type = typename async_task_void::handle_type;

        async_task(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_task<void>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start is a dummy function for eager start coroutines, but it required because
         * of its call in when_all and when_any.
         */
        void start()
        {
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
                    print(PRI2, "%p: async_task<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: async_task<void>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro.promise().m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: async_task<void>::await_resume()\n", this);
                }

            private:
                async_task& m_async_task;
            };

            return awaiter{ *this };
        }
       
        struct promise_type : public async_task_void::promise_type
        {
            friend class async_task;
            
            auto get_return_object()
            {
                print(PRI2, "%p: async_task<void>::promise_type::get_return_object()\n", this);
                return async_task<void>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task<void>::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }
        };
    };
	
    template<>
    class async_ltask<void> : public async_task_void
    {
    public:
	    using handle_type = typename async_task_void::handle_type;

        async_ltask(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_ltask<void>::async_ltask(handle_type h)\n", this);
        }
		
        /**
         * @brief start starts a lazy coroutine.
         */
        void start()
        {
            this->baseStart();
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
                    print(PRI2, "%p: async_ltask<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
				    m_async_ltask.m_coro.resume();
                    print(PRI2, "%p: async_ltask<void>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_ltask.m_coro.promise().m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: async_ltask<void>::await_resume()\n", this);
                }

            private:
                async_ltask& m_async_ltask;
            };

            return awaiter{ *this };
        }

        struct promise_type : public async_task_void::promise_type
        {
            friend class async_ltask;
            
            auto get_return_object()
            {
                print(PRI2, "%p: async_ltask<void>::promise_type::get_return_object()\n", this);
                return async_ltask<void>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask<void>::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }
        };
    };

}

#endif
