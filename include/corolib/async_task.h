/**
 * @file async_task.h
 * @brief
 * async_task<TYPE> defines an eager awaitable type: its promise_type::initial_suspend function returns std::suspend_never{}.
 * async_ltask<TYPE> defines a lazy awaitable type: its promise_type::initial_suspend function returns std::suspend_always{}.
 * (See the note below for an explanation of eager and lazy.)
 * 
 * Both types include a promise_type, so they can be used as the return type of a coroutine.
 * 
 * Both types also define operator co_await, so another coroutine can co_await the async_task<TYPE> or async_ltask<TYPE> object.
 * The TYPE in async_task<TYPE> or async_ltask<TYPE> corresponds to the "real" return type of the coroutine
 * (the type the user is interested in).
 * 
 * Eager versus lazy
 * 1) In an eager start coroutine, the coroutine passes the initial suspension point
 *    and enters immediately the user-written part of the coroutine.
 * 2) In a lazy start coroutine, the coroutine is suspended at the initial suspension point;
 *    the application has to call co_await on the coroutine return object to enter the user-written part of the coroutine.
 * 
 * Class hierarchy
 * 
 * async_task<TYPE> and async_ltask<TYPE> are derived from a common base class async_task_base<TYPE>.
 * async_task<void> and async_ltask<void> are derived from a common base class async_task_void.
 * 
 * Avoiding memory leaks using the asymmetric transfer approach
 * The reader is referred to ../../reading/Avoiding_memory_leaks.md for further info on this subject.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _ASYNC_TASK_H_
#define _ASYNC_TASK_H_

#include <coroutine>
#include <exception>

#include "print.h"
#include "tracker.h"
#include "semaphore.h"
#include "async_base.h"
#include "when_all_counter.h"
#include "when_any_one.h"

// Note: The reader is referred to ../../reading/Avoiding_memory_leaks.md for further info.

#define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN 1
#define USE_FINAL_AWAITER1 0
#define USE_RESULT_FROM_COROUTINE_OBJECT 0

#define USE_FINAL_AWAITER2 1

// Asymmetric transfer
// Possible combinations:                           1   2   3   4   5   6
// ----------------------------------------------------------------------
// #define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN    0   0   1   1   1   1
// #define USE_FINAL_AWAITER1                       0   1   0   1   0   1
// #define USE_RESULT_FROM_COROUTINE_OBJECT         0   0   0   0   1   1
//
// #define USE_FINAL_AWAITER1                       0   0   0   0   0   0
// 
// To use the original implementation, set all 4 compiler directives to 0.
// Observe that many promise_type objects are still present when leaving the application.
// 
// The combinations with USE_FINAL_AWAITER1 enabled display unreliable behavior 
// in multi-threaded applications. (Currently corolib does not use any atomics.)
// 
// USE_RESULT_FROM_COROUTINE_OBJECT = 1 requires USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN = 1
// 
// To use symmetric transfer, set USE_FINAL_AWAITER2 to 1 and USE_FINAL_AWAITER1 to 0.
// In addition, USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN can be set to 1.
// There is no need to set USE_RESULT_FROM_COROUTINE_OBJECT to 1.

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
#include <assert.h>
#endif

namespace corolib
{
    template<typename TYPE>
    class async_task_base : public async_base, private coroutine_tracker
    {
    public:
	
        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        async_task_base(const async_task_base& s) = delete;

        async_task_base(async_task_base&& s)
            : m_coro_handle{s.m_coro_handle}
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_value{s.m_value}
            , m_ready{s.m_ready}
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(async_task_base&& s)\n", this);
            s.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            s.m_value = {};
            s.m_ready = false;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = s.m_promise_type;
            m_promise_valid = s.m_promise_valid;
            s.m_promise_type = nullptr;
            s.m_promise_valid = false;
#endif
        }

        ~async_task_base()
        {
            print(PRI2, "%p: async_task_base::~async_task_base()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            coroutine_destructor_admin();
#endif
            if (m_coro_handle)  // Does the coroutine_handle still contain a valid pointer to the coroutine frame/state?
            {
                if (m_coro_handle.done()) {     // Has the coroutine reached the final suspend point (and cleared the "resume" function pointer)?
                    // Yes
                    ++tracker_obj.nr_dying_coroutines_handle_done;
                    m_coro_handle.destroy();    // Call "destroy" function
                }
                else {
                    print(PRI1, "%p: async_task_base::~async_task_base(): m_coro_handle.done() returned false\n", this);
                    ++tracker_obj.nr_dying_coroutines_handle_not_done;
                }
            }
        }

        async_task_base(handle_type h)
            : m_coro_handle(h)
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_value{}
            , m_ready{false}
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(handle_type h): promise = %p\n", this, &m_coro_handle.promise());
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_coro_handle.promise().link_coroutine_object(this);
#endif
        }

        async_task_base& operator = (const async_task_base&) = delete;

        async_task_base& operator = (async_task_base&& s)
        {
            print(PRI2, "%p: async_task_base::async_task_base = (async_task_base&& s)\n", this);
            m_coro_handle = s.m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            m_value = s.m_value;
            m_ready = s.m_ready;
#endif
            s.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            s.m_value = {};
            s.m_ready = false;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = s.m_promise_type;
            m_promise_valid = s.m_promise_valid;
            s.m_promise_type = nullptr;
            s.m_promise_valid = false;
#endif
            return *this;
        }

        /**
         * @brief Starts a lazy coroutine.
         * Should be a member function of async_ltask, but g++ does not find m_coro_handle.
         */
        void baseStart()
        {
            m_coro_handle.resume();
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
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            get_result_admin();
#endif
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: async_task_base::get_result(): m_coro_handle.promise().m_ready = %d\n", this, m_coro_handle.promise().m_ready);
            if (!m_coro_handle.promise().m_ready)
#else
            print(PRI2, "%p: async_task_base::get_result(): m_ready = %d\n", this, m_ready);
            if (!m_ready)
#endif
            {
                if (waitIfNotReady)
                {
                    print(PRI2, "%p: async_task_base::get_result(): before m_coro_handle.promise().m_sema.wait();\n", this);
                    m_coro_handle.promise().m_wait_for_signal = true;
                    m_coro_handle.promise().m_sema.wait();
                    print(PRI2, "%p: async_task_base::get_result(): after m_coro_handle.promise().m_sema.wait();\n", this);
                }
                else
                {
                    print(PRI1, "%p: async_task_base::get_result(): m_ready == false: returning {} value!\n", this);
                    return {};
                }
            }
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: async_task_base::get_result(): return m_coro_handle.promise().m_value;\n", this);
            return m_coro_handle.promise().m_value;
#else
            print(PRI2, "%p: async_task_base::get_result(): return m_value;\n", this);
            return m_value; 
#endif
        }

        bool is_ready() override
        {
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: void async_task_base::is_ready() returns %d\n", this, m_coro_handle.promise().m_ready);
            return m_coro_handle.promise().m_ready;
#else
            print(PRI2, "%p: void async_task_base::is_ready() returns %d\n", this, m_ready);
            return m_ready;
#endif
        }

        /**
         * @brief called from the constructors and destructor of when_all
         *
         */
        void setCounter(when_all_counter* ctr) override
        {
            print(PRI2, "%p: void m_async_task_base::setCounter(%p)\n", this, ctr);
            m_coro_handle.promise().m_ctr = ctr;
        }

        /**
		 * @brief called from the constructors and destructor of when_any
         *
         */
        void setWaitAny(when_any_one* waitany) override
        {
            print(PRI2, "%p: void m_async_task_base::setWaitAny(%p)\n", this, waitany);
            m_coro_handle.promise().m_waitany = waitany;
        }

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        void link_promise_type(promise_type* pt)
        {
            m_promise_type = pt;
            m_promise_valid = true;
        }

        void unlink_promise_type()
        {
            m_promise_valid = false;
        }

        void coroutine_destructor_admin()
        {
            print(PRI2, "%p: async_task_base::coroutine_destructor_admin(): promise = %p (valid = %d)\n", 
                        this, m_promise_type, m_promise_valid);
            if (m_promise_valid)
            {
                m_promise_type->unlink_coroutine_object();
                ++tracker_obj.nr_dying_coroutines_detecting_live_promise;
                assert(&m_coro_handle.promise() == m_promise_type);
            }
            else
            {
                ++tracker_obj.nr_dying_coroutines_detecting_dead_promise;
            }
        }

        void get_result_admin()
        {
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            if (!m_promise_valid)
            {
                print(PRI1, "%p: async_task_base::get_result_admin(): promise %p invalid!!!\n", 
                            this, m_promise_type);
                ++tracker_obj.nr_access_errors;
            }
#else
            if (!m_promise_valid)
            {
                print(PRI1, "%p: async_task_base::get_result_admin(): promise %p invalid!!!\n", 
                            this, m_promise_type);
            }
#endif
        }
#endif

        struct promise_type: private promise_type_tracker
        {
            friend class async_task_base;

            promise_type()
                : m_awaiting(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                , m_value{}
                , m_ready(false)
#endif
                , m_wait_for_signal(false)
            {
                print(PRI2, "%p: async_task_base::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                print(PRI2, "%p: async_task_base::promise_type::~promise_type()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                promise_destructor_admin();
#endif
            }

            void return_value(TYPE v)
            {
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): begin\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_value = v;
                m_ready = true;
#else
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): m_coroutine_object = %p (m_coroutine_valid = %d)\n",
                            this, m_coroutine_object, m_coroutine_valid);
                if (m_coroutine_valid)
                {
                    m_coroutine_object->m_value = v;
                    m_coroutine_object->m_ready = true;
                }
#endif
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v):\n\t\tm_ctr = %p, m_waitany = %p, &m_awaiting = %p, m_wait_for_signal = %d\n",
                            this, m_ctr, m_waitany, &m_awaiting, m_wait_for_signal);
                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_ctr->completed();\n", this);
                    m_awaiting = m_ctr->completed();
#if !USE_FINAL_AWAITER2
                    m_awaiting.resume();
#endif
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_waitany->completed();\n", this);
                    m_awaiting = m_waitany->completed();
#if !USE_FINAL_AWAITER2
                    m_awaiting.resume();
#endif
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_waitany->completed();\n", this);
                    return;
                }
#if !USE_FINAL_AWAITER2
                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_awaiting.resume();\n", this);
                }
#endif
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): before m_sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): after m_sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): end\n", this);
            }
#if !USE_FINAL_AWAITER1 && !USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task_base::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }
#endif
            void unhandled_exception()
            {
                print(PRI1, "%p: async_task_base::promise_type::unhandled_exception()\n", this);
                m_exception = std::current_exception();
#if 0
                print(PRI1, "%p: async_task_base::promise_type::unhandled_exception(): rethrow exception\n", this);
                std::rethrow_exception(m_exception);
#endif
            }
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            TYPE get_result_promise()
            {
                print(PRI2, "%p: async_task_base::promise_type::get_result_promise()\n", this);
                if (m_exception != nullptr)
                {
                    print(PRI1, "%p: async_task_base::promise_type::get_result_promise(): throwing exception\n", this);
                    std::rethrow_exception(m_exception);
                }
                return m_value;
            }
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            void link_coroutine_object(async_task_base* coroutine_object)
            {
                m_coroutine_object = coroutine_object;
                m_coroutine_valid = true;
            }

            void unlink_coroutine_object()
            {
                m_coroutine_valid = false;
            }

            void promise_destructor_admin()
            {
                print(PRI2, "%p: async_task_base::promise_type::promise_destructor_admin(): m_coroutine_object = %p (m_coroutine_valid = %d)\n", 
                            this, m_coroutine_object, m_coroutine_valid);
                if (m_coroutine_valid)
                {
                    m_coroutine_object->unlink_promise_type();
                    ++tracker_obj.nr_dying_promises_detecting_live_coroutine;
                }
                else
                {
                    ++tracker_obj.nr_dying_promises_detecting_dead_coroutine;
                }
            }
#endif

        public:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            TYPE m_value;
            bool m_ready;
#endif
            bool m_wait_for_signal;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            async_task_base* m_coroutine_object = nullptr;
            bool m_coroutine_valid = false;
#endif
        private:
            std::exception_ptr m_exception;
        };

    protected:
        handle_type m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
        TYPE m_value;
        bool m_ready;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        promise_type* m_promise_type = nullptr;
        bool m_promise_valid = false;
#endif
    };


    template<typename TYPE>
    class async_task : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        async_task(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_task<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start is a dummy function for eager start coroutines, but it required because
         * of its call in when_all and when_any.
         */
        void start() override
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
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const bool ready = m_async_task.m_coro_handle.promise().m_ready;
#else
                    const bool ready = m_async_task.m_ready;
#endif
                    print(PRI2, "%p: async_task<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: async_task<TYPE>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro_handle.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: async_task<TYPE>::await_resume()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const TYPE r = m_async_task.m_coro_handle.promise().get_result_promise();
#else
                    const TYPE r = m_async_task.m_value;
#endif
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
                auto ret = async_task<TYPE>{handle_type::from_promise(*this)};
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                ret.link_promise_type(this);
#endif
                return ret;
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend()\n", this);
                    promise_type& promise = h.promise();
                    bool result = false;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    result = !promise.m_ready;
#else 
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                    async_task_base<TYPE>* coroutine_object = promise.m_coroutine_object;
                    bool coroutine_valid = promise.m_coroutine_valid;
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend(): coroutine_object = %p (valid = %d)\n",
                                this, coroutine_object, coroutine_valid);
                    if (coroutine_valid)
                    {
                        bool is_ready = coroutine_object->is_ready();
                        print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, is_ready);
                        result = !is_ready;
                    }
#endif
#endif
                    result ? ++tracker_obj.nr_final_awaiters_await_suspend_returning_true :
                             ++tracker_obj.nr_final_awaiters_await_suspend_returning_false;

                    if (result)
                        print(PRI1, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend() suspends the coroutine\n", this);
                    return result;
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_resume()\n", this);
                }
            };

            struct final_awaiter2 : public final_awaiter_tracker 
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_ready()\n", this);
                    return false;
                }

                std::coroutine_handle<> await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_suspend()\n", this);
                    print(PRI2, "h.promise().m_awaiting = %p\n", h.promise().m_awaiting);
                    if (h.promise().m_awaiting)
                        return h.promise().m_awaiting;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER1
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#endif
        };

    };

    template<typename TYPE>
    class async_ltask : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        async_ltask(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_ltask<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start starts a lazy coroutine.
         * 
         */
        void start() override
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
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const bool ready = m_async_ltask.m_coro_handle.promise().m_ready;
#else
                    const bool ready = m_async_ltask.m_ready;
#endif
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    m_async_ltask.m_coro_handle.resume();
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_ltask.m_coro_handle.promise().m_awaiting = awaiting;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_resume()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const TYPE r = m_async_ltask.m_coro_handle.promise().get_result_promise();
#else
                    const TYPE r = m_async_ltask.m_value;
#endif
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
                auto ret = async_ltask<TYPE>{handle_type::from_promise(*this)};
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                ret.link_promise_type(this);
#endif
                return ret;
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend()\n", this);
                    promise_type& promise = h.promise();
                    bool result = false;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    result = !promise.m_ready;
#else
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                    async_task_base<TYPE>* coroutine_object = promise.m_coroutine_object;
                    bool coroutine_valid = promise.m_coroutine_valid;
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend(): coroutine_object = %p (valid = %d)\n",
                                this, coroutine_object, coroutine_valid);
                    if (coroutine_valid)
                    {
                        bool is_ready = coroutine_object->is_ready();
                        print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, is_ready);
                        result = !is_ready;
                    }
#endif
#endif
                    result ? ++tracker_obj.nr_final_awaiters_await_suspend_returning_true : 
                             ++tracker_obj.nr_final_awaiters_await_suspend_returning_false;

                    if (result)
                        print(PRI1, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend() suspends the coroutine\n", this);
                    return result;
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_resume()\n", this);
                }
            };

            struct final_awaiter2 : public final_awaiter_tracker 
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter2::await_ready()\n", this);
                    return false;
                }

                std::coroutine_handle<> await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter2::await_suspend()\n", this);
                    print(PRI2, "h.promise().m_awaiting = %p\n", h.promise().m_awaiting);
                    if (h.promise().m_awaiting)
                        return h.promise().m_awaiting;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER1
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#endif
        };

    };

    // ---------------------------------------------------------------------
    // ---------------------------------------------------------------------

    class async_task_void : public async_base, private coroutine_tracker
    {
    public:

        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        async_task_void(const async_task_void& s) = delete;

        async_task_void(async_task_void&& s) noexcept
            : m_coro_handle{s.m_coro_handle}
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_ready{s.m_ready}
#endif
        {
            print(PRI2, "%p: async_task_void::async_task_void(async_task&& s)\n", this);
            s.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            s.m_ready = false;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = s.m_promise_type;
            m_promise_valid = s.m_promise_valid;
            s.m_promise_type = nullptr;
            s.m_promise_valid = false;
#endif
        }

        ~async_task_void()
        {
            print(PRI2, "%p: async_task_void::~async_task_void()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            coroutine_destructor_admin();
#endif
            if (m_coro_handle)  // Does the coroutine_handle still contain a valid pointer to the coroutine frame/state?
            {
                if (m_coro_handle.done()) {     // Has the coroutine reached the final suspend point (and cleared the "resume" function pointer)?
                    // Yes
                    ++tracker_obj.nr_dying_coroutines_handle_done;
                    m_coro_handle.destroy();    // Call the "destroy" function
                }
                else {
                    print(PRI1, "%p: async_task_void::~async_task_void(): m_coro_handle.done() returned false\n", this);
                    ++tracker_obj.nr_dying_coroutines_handle_not_done;
                }
            }
        }

        async_task_void(handle_type h)
            : m_coro_handle{h}
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_ready{false}
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(handle_type h): promise = %p\n", this, &m_coro_handle.promise());
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_coro_handle.promise().link_coroutine_object(this);
#endif
        }

        async_task_void& operator = (const async_task_void&) = delete;

        async_task_void& operator = (async_task_void&& s) noexcept
        {
            print(PRI2, "%p: async_task_void::async_task_void = (async_task&& s)\n", this);
            m_coro_handle = s.m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            m_ready = s.m_ready;
#endif
            s.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            s.m_ready = false;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = s.m_promise_type;
            m_promise_valid = s.m_promise_valid;
            s.m_promise_type = nullptr;
            s.m_promise_valid = false;
#endif
            return *this;
        }

        /**
         * @brief baseStart() starts a lazy coroutine.
         * Should be a member function of async_ltask, but g++ does not find m_coro_handle.
         */
        void baseStart()
        {
            m_coro_handle.resume();
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
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: async_task_void::wait(): m_coro_handle.promise().m_ready = %d\n", this, m_coro_handle.promise().m_ready);
            if (!m_coro_handle.promise().m_ready)
#else
            print(PRI2, "%p: async_task_void::wait(): m_ready = %d\n", this, m_ready);
            if (!m_ready)
#endif
            {
                if (waitIfNotReady)
                {
                    print(PRI2, "%p: async_task_void::wait(): before m_coro_handle.promise().m_sema.wait();\n", this);
                    m_coro_handle.promise().m_wait_for_signal = true;
                    m_coro_handle.promise().m_sema.wait();
                    print(PRI2, "%p: async_task_void::wait(): after m_coro_handle.promise().m_sema.wait();\n", this);
                }
                else
                {
                    print(PRI1, "%p: async_task_void::wait(): m_ready == false: returning without waiting for ready!\n", this);
                }
            }
        }

        bool is_ready() override
        {
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            ready_admin();
#endif
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: void async_task_void::is_ready() returns %d\n", this, m_coro_handle.promise().m_ready);
            return m_coro_handle.promise().m_ready;
#else
            print(PRI2, "%p: void async_task_void::is_ready() returns %d\n", this, m_ready);
            return m_ready;
#endif
        }

        void setCounter(when_all_counter* ctr) override
        {
            print(PRI2, "%p: void async_task_void::setCounter(%p)\n", this, ctr);
            m_coro_handle.promise().m_ctr = ctr;
        }

        void setWaitAny(when_any_one* waitany) override
        {
            print(PRI2, "%p: void async_task_void::setWaitAny(%p)\n", this, waitany);
            m_coro_handle.promise().m_waitany = waitany;
        }

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        void link_promise_type(promise_type* pt)
        {
            m_promise_type = pt;
            m_promise_valid = true;
        }

        void unlink_promise_type()
        {
            m_promise_valid = false;
        }

        void coroutine_destructor_admin()
        {
            print(PRI2, "%p: async_task_void::coroutine_destructor_admin(): promise = %p (valid = %d)\n",
                        this, m_promise_type, m_promise_valid);
            if (m_promise_valid)
            {
                m_promise_type->unlink_coroutine_object();
                ++tracker_obj.nr_dying_coroutines_detecting_live_promise;
                assert(&m_coro_handle.promise() == m_promise_type);
            }
            else
            {
                ++tracker_obj.nr_dying_coroutines_detecting_dead_promise;
            }
        }

        void ready_admin()
        {
            if (!m_promise_valid)
            {
                print(PRI1, "%p: async_task_void::ready_admin(): retrieving value from destructed promise %p!!!\n",
                            this, m_promise_type);
                ++tracker_obj.nr_access_errors;
            }
        }
#endif

        struct promise_type: private promise_type_tracker
        {
            friend class async_task_void;

            promise_type()
                : m_awaiting(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                , m_ready(false)
#endif
                , m_wait_for_signal(false)
            {
                print(PRI2, "%p: async_task_void::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                print(PRI2, "%p: async_task_void::promise_type::~promise_type()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                promise_destructor_admin();
#endif
            }

            void return_void()
            {
                print(PRI2, "%p: async_task_void::promise_type::return_void(): begin\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                print(PRI2, "%p: async_task_void::promise_type::return_void(): m_coroutine_object = %p (m_coroutine_valid = %d)\n",
                                 this, m_coroutine_object, m_coroutine_valid);
#endif
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_ready = true;
#else
                if (m_coroutine_valid)
                    m_coroutine_object->m_ready = true;
#endif
                print(PRI2, "%p: async_task_void::promise_type::return_value(TYPE v):\n\t\tm_ctr = %p, m_waitany = %p, &m_awaiting = %p, m_wait_for_signal = %d\n",
                                this, m_ctr, m_waitany, &m_awaiting, m_wait_for_signal);
                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_ctr->completed();\n", this);
                    m_awaiting = m_ctr->completed();
#if !USE_FINAL_AWAITER2
                    m_awaiting.resume();
#endif
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_ctr->completed();\n", this);
                    return;
                }
                if (m_waitany)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_waitany->completed();\n", this);
                    m_awaiting = m_waitany->completed();
#if !USE_FINAL_AWAITER2
                    m_awaiting.resume();
#endif
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_waitany->completed();\n", this);
                    return;
                }
#if !USE_FINAL_AWAITER2
                if (m_awaiting)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_awaiting.resume();\n", this);
                    m_awaiting.resume();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_awaiting.resume();\n", this);
                }
#endif
                if (m_wait_for_signal)
                {
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): before m_sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_void::promise_type::return_void(): after m_sema.signal();\n", this);
                }
                print(PRI2, "%p: async_task_void::promise_type::return_void(): end\n", this);
            }
#if !USE_FINAL_AWAITER1 && !USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task_void::promise_type::final_suspend()\n", this);
                return std::suspend_always{};
            }
#endif
            void unhandled_exception()
            {
                print(PRI1, "%p: async_task_void::promise_type::unhandled_exception()\n", this);
                m_exception = std::current_exception();
            }

            void get_result_promise()
            {
                print(PRI2, "%p: async_task_void::promise_type::get_result_promise()\n", this);
                if (m_exception != nullptr)
                {
                    print(PRI1, "%p: async_task_void::promise_type::get_result_promise(): throwing exception\n", this);
                    std::rethrow_exception(m_exception);
                }
            }

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            void link_coroutine_object(async_task_void* coroutine_object)
            {
                m_coroutine_object = coroutine_object;
                m_coroutine_valid = true;
            }

            void unlink_coroutine_object()
            {
                m_coroutine_valid = false;
            }

            void promise_destructor_admin()
            {
                print(PRI2, "%p: async_task_void::promise_type::promise_destructor_admin(): m_coroutine_object = %p (m_coroutine_valid = %d)\n", 
                            this, m_coroutine_object, m_coroutine_valid);
                if (m_coroutine_valid)
                {
                    m_coroutine_object->unlink_promise_type();
                    ++tracker_obj.nr_dying_promises_detecting_live_coroutine;
                }
                else
                {
                    ++tracker_obj.nr_dying_promises_detecting_dead_coroutine;
                }
            }
#endif

        public:
            std::coroutine_handle<> m_awaiting;
            Semaphore m_sema;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            bool m_ready;
#endif
            bool m_wait_for_signal;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            async_task_void* m_coroutine_object = nullptr;
            bool m_coroutine_valid = false;
#endif
        private:
            std::exception_ptr m_exception;
        };

    protected:
        handle_type m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
        bool m_ready;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        promise_type* m_promise_type = nullptr;
        bool m_promise_valid = false;
#endif
    };

    template<>
    class async_task<void> : public async_task_void
    {
    public:
	
	    using handle_type = typename async_task_void::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        async_task(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_task<void>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief start is a dummy function for eager start coroutines, but it required because
         * of its call in when_all and when_any.
         */
        void start() override
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
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const bool ready = m_async_task.m_coro_handle.promise().m_ready;
#else
                    const bool ready = m_async_task.m_ready;
#endif
                    print(PRI2, "%p: async_task<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: async_task<void>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_task.m_coro_handle.promise().m_awaiting = awaiting;
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
                auto ret = async_task<void>{handle_type::from_promise(*this)};
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                ret.link_promise_type(this);
#endif
                return ret;
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task<void>::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend()\n", this);
                    promise_type& promise = h.promise();
                    bool result = false;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    result = !promise.m_ready;
#else
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                    async_task_void* coroutine_object = promise.m_coroutine_object;
                    bool coroutine_valid = promise.m_coroutine_valid;
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend(): coroutine_object = %p (valid = %d)\n",
                                this, coroutine_object, coroutine_valid);
                    if (coroutine_valid)
                    {
                        bool is_ready = coroutine_object->is_ready();
                        print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, is_ready);
                        result = !is_ready;
                    }
#endif
#endif
                    result ? ++tracker_obj.nr_final_awaiters_await_suspend_returning_true :
                             ++tracker_obj.nr_final_awaiters_await_suspend_returning_false;

                    if (result)
                        print(PRI1, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend() suspends the coroutine\n", this);
                    return result;
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_resume()\n", this);
                }
            };

            struct final_awaiter2 : public final_awaiter_tracker 
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter2::await_ready()\n", this);
                    return false;
                }

                std::coroutine_handle<> await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter2::await_suspend()\n", this);
                    print(PRI2, "h.promise().m_awaiting = %p\n", h.promise().m_awaiting);
                    if (h.promise().m_awaiting)
                        return h.promise().m_awaiting;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };
#if USE_FINAL_AWAITER1
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#endif
        };

    };
	
    template<>
    class async_ltask<void> : public async_task_void
    {
    public:
	    using handle_type = typename async_task_void::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        async_ltask(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_ltask<void>::async_ltask(handle_type h)\n", this);
        }
		
        /**
         * @brief start starts a lazy coroutine.
         */
        void start() override
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
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const bool ready = m_async_ltask.m_coro_handle.promise().m_ready;
#else
                    const bool ready = m_async_ltask.m_ready;
#endif
                    print(PRI2, "%p: async_ltask<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
				    m_async_ltask.m_coro_handle.resume();
                    print(PRI2, "%p: async_ltask<void>::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_async_ltask.m_coro_handle.promise().m_awaiting = awaiting;
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
                auto ret = async_ltask<void>{handle_type::from_promise(*this)};
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                ret.link_promise_type(this);
#endif
                return ret;
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask<void>::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend()\n", this);
                    promise_type& promise = h.promise();
                    bool result = false;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    result = !promise.m_ready;
#else
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                    async_task_void* coroutine_object = promise.m_coroutine_object;
                    bool coroutine_valid = promise.m_coroutine_valid;
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend(): coroutine_object = %p (valid = %d)\n", 
                                this, coroutine_object, coroutine_valid);
                    if (coroutine_valid)
                    {
                        bool is_ready = coroutine_object->is_ready();
                        print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend(): m_ready = %d\n", this, is_ready);
                        result = !is_ready;
                    }
#endif
#endif
                    result ? ++tracker_obj.nr_final_awaiters_await_suspend_returning_true :
                             ++tracker_obj.nr_final_awaiters_await_suspend_returning_false;

                    if (result)
                        print(PRI1, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend() suspends the coroutine\n", this);
                    return result;
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_resume()\n", this);
                }
            };

            struct final_awaiter2 : public final_awaiter_tracker 
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter2::await_ready()\n", this);
                    return false;
                }

                std::coroutine_handle<> await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter2::await_suspend()\n", this);
                    print(PRI2, "h.promise().m_awaiting = %p\n", h.promise().m_awaiting);
                    if (h.promise().m_awaiting)
                        return h.promise().m_awaiting;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER1
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER2
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#endif
        };

    };

}

#endif
