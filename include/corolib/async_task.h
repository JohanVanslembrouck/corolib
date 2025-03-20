/**
 * @file async_task.h
 * @brief
 * async_task<TYPE> defines an eager awaitable type: its promise_type::initial_suspend function returns std::suspend_never{}.
 * async_ltask<TYPE> defines a lazy awaitable type: its promise_type::initial_suspend function returns std::suspend_always{}.
 * (See the note below for an explanation of eager and lazy.)
 * 
 * Both types include a promise_type, so they can be used as the return type of a coroutine.
 * 
 * Both types also define operator co_await, so that another coroutine can co_await the async_task<TYPE> or async_ltask<TYPE> object.
 * The TYPE in async_task<TYPE> or async_ltask<TYPE> corresponds to the "real" return type of the coroutine
 * (the type the user is interested in).
 * 
 * Eager versus lazy
 * 1) In an eager start coroutine, the coroutine passes the initial suspension point
 *    and enters immediately the user-written part of the coroutine.
 * 2) In a lazy start coroutine, the coroutine is suspended at the initial suspension point;
 *    the application has to call co_await on the coroutine return object to enter the user-written part of the coroutine.
 * 
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
                     template<typename TYPE>
                     class async_task_base
                             ^
                             |
                             |
             ----------------------------------
             |                                |
             |                                |
     template<typename TYPE>             template<typename TYPE>
     class async_task                    class async_ltask



                     class async_base
                             ^
                             |
                             |
                     class async_task_void
                             ^
                             |
                             |
             ----------------------------------
             |                                |
             |                                |
     template<>                          template<>
     class async_task<void>              class async_ltask<void>

The second class hierarchy (for void) is independent from the one for non-void types,
apart from the common base class async_base.

 */

#ifndef _ASYNC_TASK_H_
#define _ASYNC_TASK_H_

#include <coroutine>
#include <exception>
#include <assert.h>

#if USE_IN_MT_APPS
#include <atomic>
#endif

#include "print.h"
#include "tracker.h"
#include "semaphore.h"
#include "async_base.h"
#include "when_all_counter.h"
#include "when_any_one.h"

// Some compilers require the copy or move constructor to be present
// to be able to return an object from get_return_object().
// For other compilers (MSVC, g++11) DECLARE_MOVE_CONSTRUCTORS_AS_DELETED can be set to 1.
// However, if applications make use of assignments,
// e.g. as in: tasks[i] = std::move(clientTask());
// set the next two compiler directives to 0
#define DECLARE_MOVE_CONSTRUCTORS_AS_DELETED 0
#define DECLARE_MOVE_ASSIGMENT_OPERATORS_AS_DELETED 0

#define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN 1
#define USE_RESULT_FROM_COROUTINE_OBJECT 0

#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL 0
#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE 1

/**
Explanation
===========

Asymmetric transfer
Combinations: (impossible cases are marked X)            1   2   X   X   5   6   7   8
--------------------------------------------------------------------------------------
#define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN            0   0   0   0   1   1   1   1
#define USE_RESULT_FROM_COROUTINE_OBJECT                 0   0   1   1   0   0   1   1
        USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID     1   0   1   0   1   0   1   0   (implicit: 1 if the following two are 0)
#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL     0   1   0   1   0   1   0   1
#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE   0   0   0   0   0   0   0   0

Symmetric transfer
Combinations: (impossible cases are marked X)            9   X   X   X  13   X  15   X
--------------------------------------------------------------------------------------
#define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN            0   0   0   0   1   1   1   1
#define USE_RESULT_FROM_COROUTINE_OBJECT                 0   0   1   1   0   0   1   1
        USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID     0   0   0   0   0   0   0   0   (implicit)
#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL     0   1   0   1   0   1   0   1
#define USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE   1   1   1   1   1   1   1   1

USE_RESULT_FROM_COROUTINE_OBJECT = 1 requires USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN = 1.

USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL and USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE
cannot be 1 at the same time.

To use symmetric transfer, set USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE to 1 
and USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL to 0.

In addition, USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN can be set to 1.
There is no need to set USE_RESULT_FROM_COROUTINE_OBJECT to 1.

With USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN = 0, the values in the tracker columns c>p and c<p
(displayed in the table when leaving an application) are always 0.

In conclusion: use combinations 9, 13 or 15 only.

The reader is also referred to ../../reading/Avoiding_memory_leaks.md for further info.

*/


/**
Overview of possible and used combinations for await_ready, await_suspend and await_resume
==========================================================================================

type                            return type                 usage in corolib
        await_* function                                    numbers: see tables above 
------------------------------------------------------------------------------------------------
async_task | async_ltask
    awaiter
        await_ready             bool                        return value is calculated
        await_suspend           void                        async_task: (no resumption from here)
                                                            async_ltask: m_async_ltask.m_coro_handle.resume();
                                bool                        this variant is not used
                                std::coroutine_handle<>     this variqnt is not used
        await_resume            TYPE or void

    promise_type
        initial_awaiter                                     std::suspend_never (async_task)
                                                            std::suspend_always (async_ltask)
            await_ready         bool                        true: async_task
                                                            false: async_ltask
            await_suspend       void
                                bool                        this variant is not used
                                std::coroutine_handle<>     this variant is not used
            await_resume        void

        final_awaiter
            await_ready         bool                        always returns false
            await_suspend       void                        1, 5, 7 
                                bool                        2, 6, 8
                                std::coroutine_handle<>     9, 12, 15
            await_resume        void
*/

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
#include <assert.h>
#endif

namespace corolib
{
    /**
     * @brief class result_t
     */
    template<typename TYPE>
    class result_t
    {
    private:
        enum class completion_status : char
        {
            INITIAL = 0,
            COMPLETED,
            WAIT_FOR_SEMAPHORE_RELEASE,
        };

    public:
        result_t()
            : m_value{}
            , m_exception{nullptr}
            , m_ready{completion_status::INITIAL}
            , m_wait_for_semaphore_release{false}
        {
            print(PRI2, "%p: result_t::result_t();\n", this);
        }

        ~result_t()
        {
            print(PRI2, "%p: result_t::~result_t();\n", this);

            m_value = {};
            m_exception = nullptr;
            m_ready = completion_status::INITIAL;
            m_wait_for_semaphore_release = false;
        }

        result_t(const result_t&) = delete;
 
        result_t(result_t&& other)
            : m_value(other.m_value)
            , m_exception(other.m_exception)
            , m_ready(other.m_ready)
            , m_wait_for_semaphore_release(other.m_wait_for_semaphore_release)
        {
            print(PRI2, "%p: result_t::result_t(result_t&& other);\n", this);
            other.m_exception = nullptr;
            other.m_ready = completion_status::INITIAL;
            other.m_wait_for_semaphore_release = false;
        }

        result_t& operator = (const result_t&) = delete;
 
        result_t& operator = (result_t&& other) noexcept
        {
            print(PRI2, "%p: result_t::operator = (result_t&& other);\n", this);
            m_value = other.m_value;
            m_exception = other.m_exception;
            m_ready = other.m_ready;
            m_wait_for_semaphore_release = other.m_wait_for_semaphore_release;
            other.m_exception = nullptr;
            other.m_ready = completion_status::INITIAL;
            other.m_wait_for_semaphore_release = false;
            return *this;
        }

        void set_value(const TYPE& value)
        {
            print(PRI2, "%p: result_t::set_value(const TYPE& value);\n", this);
            m_value = value;
            
#if USE_IN_MT_APPS
            completion_status expected = completion_status::INITIAL;
            if (!m_ready.compare_exchange_strong(expected, completion_status::COMPLETED)) {
                if (expected != completion_status::WAIT_FOR_SEMAPHORE_RELEASE)
                    print(PRI1, "%p: result_t::set_value(const TYPE& value): expected = %d != completion_status::WAIT_FOR_SEMAPHORE_RELEASE\n", this);
                //assert(expected == completion_status::WAIT_FOR_SEMAPHORE_RELEASE);
                m_ready = completion_status::COMPLETED;
                m_wait_for_semaphore_release = true;
            }
#else
            if (m_ready != completion_status::INITIAL) {
                assert(m_ready == completion_status::WAIT_FOR_SEMAPHORE_RELEASE);
                m_ready = completion_status::COMPLETED;
                m_wait_for_semaphore_release = true;
            }
            else
                m_ready = completion_status::COMPLETED;
#endif
        }

        void set_exception(std::exception_ptr exception)
        {
            m_exception = exception;
            
#if USE_IN_MT_APPS
            completion_status expected = completion_status::INITIAL;
            if (!m_ready.compare_exchange_strong(expected, completion_status::COMPLETED)) {
                assert(expected == completion_status::WAIT_FOR_SEMAPHORE_RELEASE);
                m_ready = completion_status::COMPLETED;
                m_wait_for_semaphore_release = true;
            }
#else
            if (m_ready != completion_status::INITIAL) {
                assert(m_ready == completion_status::WAIT_FOR_SEMAPHORE_RELEASE);
                m_ready = completion_status::COMPLETED;
                m_wait_for_semaphore_release = true;
            }
            else
                m_ready = completion_status::COMPLETED;
#endif
        }
        
        bool is_ready()
        {
            bool ready = (m_ready == completion_status::COMPLETED);
            print(PRI2, "%p: result_t::is_ready(): return %d;\n", this, ready);
            return ready;
        }

        void reset()
        {
            m_ready = completion_status::INITIAL;
        }

        bool wait_for_result()
        {
            bool wait = false;
#if USE_IN_MT_APPS
            completion_status expected = completion_status::INITIAL;
            if (m_ready.compare_exchange_strong(expected, completion_status::WAIT_FOR_SEMAPHORE_RELEASE)) {
                wait = true;
            }
            else {
                if (expected != completion_status::COMPLETED)
                    print(PRI1, "%p: result_t::wait_for_result(): expected = %d != completion_status::COMPLETED\n", this);
                assert(expected == completion_status::COMPLETED);
            }
#else
            if (m_ready == completion_status::INITIAL) {
                m_ready = completion_status::WAIT_FOR_SEMAPHORE_RELEASE;
                wait = true;
            }
            else {
                assert(m_ready == completion_status::COMPLETED);
            }
#endif
            print(PRI2, "%p: result_t::wait_for_result(): return %d;\n", this, wait);
            return wait;
        }
        
        TYPE retrieve_result()
        {
            if (m_ready == completion_status::INITIAL)
                print(PRI1, "%p: result_t::retrieve_result(): m_ready == INITIAL!!!\n", this);
            if (m_exception != nullptr)
            {
                print(PRI1, "%p: result_t::retrieve_result(): std::rethrow_exception(m_exception);\n", this);
                std::rethrow_exception(m_exception);
            }
            print(PRI2, "%p: result_t::retrieve_result(): return m_value;\n", this);
            return m_value;
        }
        
        bool wait_for_semaphore_release()
        {
            print(PRI2, "%p: result_t::wait_for_semaphore_release(): return %d;\n", this, m_wait_for_semaphore_release);
            return m_wait_for_semaphore_release;
        }
        
    private:
        TYPE m_value{};
        std::exception_ptr m_exception;
#if USE_IN_MT_APPS
        std::atomic<completion_status> m_ready{0};
#else
        completion_status m_ready{0};
#endif
        // m_wait_for_semaphore_release is accessed from a single thread only (the completion thread).
        // Therefore it does not have to be atomic.
        bool m_wait_for_semaphore_release{false};
    };


    /**
     * @brief class async_task_base
     */
    template<typename TYPE>
    class async_task_base : public async_base, private coroutine_tracker
    {
    public:
    
        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        /**
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_task_base(handle_type h)
            : m_coro_handle(h)
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ }
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(handle_type h): promise = %p\n", this, &m_coro_handle.promise());
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_coro_handle.promise().link_coroutine_object(this);
#endif
        }

        /**
         * @brief this constructor can be used to construct a "placeholder" async_task object
         * to which a "real" async_task object (constructed by the previous constuctor)
         * will be assigned.
         */
        async_task_base()
            : m_coro_handle(nullptr)
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ }
#endif
        {
        }

        async_task_base(const async_task_base&) = delete;
#if DECLARE_MOVE_CONSTRUCTORS_AS_DELETED
        async_task_base(async_task_base&&) = delete;
#else
        async_task_base(async_task_base&& other)
            : m_coro_handle{ other.m_coro_handle}
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ std::move(other.m_result) }
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(async_task_base&& other)\n", this);
            other.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            other.m_result = { };
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = other.m_promise_type;
            m_promise_valid = other.m_promise_valid;
            other.m_promise_type = nullptr;
            other.m_promise_valid = false;
#endif
        }
#endif

        void destroy_coroutine_frame()
        {
            if (m_coro_handle)  // Does the coroutine_handle still contain a valid pointer to the coroutine frame/state?
            {
                if (m_coro_handle.done()) {     // Has the coroutine reached the final suspend point (and cleared the "resume" function pointer)?
                    // Yes
                    ++tracker_obj.nr_dying_coroutines_handle_done;
                    m_coro_handle.destroy();    // Call "destroy" function
                }
                else {
                    print(PRI1, "%p: async_task_base::destroy_coroutine_frame(): m_coro_handle.done() returned false\n", this);
                    ++tracker_obj.nr_dying_coroutines_handle_not_done;
                }
            }
        }

        ~async_task_base()
        {
            print(PRI2, "%p: async_task_base::~async_task_base()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            coroutine_destructor_admin();
#endif
            destroy_coroutine_frame();
        }

        async_task_base& operator = (const async_task_base&) = delete;
#if DECLARE_MOVE_ASSIGMENT_OPERATORS_AS_DELETED
        async_task_base& operator = (async_task_base&&) = delete;
#else
        async_task_base& operator = (async_task_base&& other)
        {
            print(PRI2, "%p: async_task_base::async_task_base = (async_task_base&& other)\n", this);
            destroy_coroutine_frame();
            m_coro_handle = other.m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            m_result = std::move(other.m_result);
#endif
            other.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            other.m_result = { };
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = other.m_promise_type;
            m_promise_valid = other.m_promise_valid;
            other.m_promise_type = nullptr;
            other.m_promise_valid = false;
#endif
            return *this;
        }
#endif
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
            print(PRI2, "%p: async_task_base::get_result(%d)\n", this, waitIfNotReady);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            get_result_admin();
#endif

            if (waitIfNotReady) {
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                print(PRI2, "%p: async_task_base::get_result(): m_coro_handle.promise().m_result.wait_for_result()\n", this);
                if (m_coro_handle.promise().m_result.wait_for_result())
#else
                print(PRI2, "%p: async_task_base::get_result(): m_result.wait_for_result()\n", this);
                if (m_result.wait_for_result())
#endif
                {
                    print(PRI2, "%p: async_task_base::get_result(): m_coro_handle.promise().m_sema.wait()\n", this);
                    m_coro_handle.promise().m_sema.wait();
                }
            }

#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: async_task_base::get_result(): return m_coro_handle.promise().m_result.retrieve_result();\n", this);
            return m_coro_handle.promise().m_result.retrieve_result();
#else
            print(PRI2, "%p: async_task_base::get_result(): return m_result.retrieve_result();\n", this);
            return m_result.retrieve_result();
#endif
        }

        bool is_ready() override
        {
            bool ready = false;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            if (m_coro_handle)
                ready = m_coro_handle.promise().m_result.is_ready();
#else
            ready = m_result.is_ready();
#endif
            print(PRI2, "%p: async_task_base::is_ready() returns %d\n", this, ready);
            return ready;
        }

        void reset()
        {
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            if (m_coro_handle)
                m_coro_handle.promise().m_result.reset();
#else
            m_result.reset();
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
                : m_continuation(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                , m_result{ }
#endif
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

            /**
             * @brief inform_interested_parties is an auxiliary function that
             * informs the coroutine that references this async_task object using a when_all or a when_any
             * or a function that awaits the completion of this async_task in a different thread
             * that the async_task has completed.
             * The function is called from return_value.
             * See async_task_void::promise_type::inform_interested_parties and
             * async_operation_base::inform_interested_parties for similar functions.
             */
            void inform_interested_parties()
            {
                print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties():\n\tm_ctr = %p, m_waitany = %p, m_continuation = %p\n",
                    this, m_ctr, m_waitany, m_continuation);
                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): before m_ctr->completed();\n", this);
                    m_continuation = m_ctr->completed();
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): after m_ctr->completed();\n", this);
                }
                else if (m_waitany)
                {
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): before m_waitany->completed();\n", this);
                    m_continuation = m_waitany->completed();
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): after m_waitany->completed();\n", this);
                }
                else if (m_result.wait_for_semaphore_release())
                {
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): before m_sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_base::promise_type::inform_interested_parties(): after m_sema.signal();\n", this);
                }
            }

            void return_value(TYPE v)
            {
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): begin\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_result.set_value(v);
#else
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): m_coroutine_object = %p (m_coroutine_valid = %d)\n",
                    this, m_coroutine_object, m_coroutine_valid);
                if (m_coroutine_valid)
                {
                    m_coroutine_object->m_result.set_value(v);
                }
#endif
                inform_interested_parties();
                print(PRI2, "%p: async_task_base::promise_type::return_value(TYPE v): end\n", this);
            }

            void unhandled_exception()
            {
                print(PRI1, "%p: async_task_base::promise_type::unhandled_exception()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_result.set_exception(std::current_exception());
#else
                if (m_coroutine_valid)
                {
                    m_coroutine_object->m_result.set_exception(std::current_exception());
                }
#endif
            }

#if !USE_RESULT_FROM_COROUTINE_OBJECT
            TYPE get_result_promise()
            {
                return m_result.retrieve_result();
            }
#endif

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            void link_coroutine_object(async_task_base* coroutine_object)
            {
                m_coroutine_object = coroutine_object;
                m_coroutine_valid = true;
                m_coroutine_object->link_promise_type(this);
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
            std::coroutine_handle<> m_continuation;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            Semaphore m_sema;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            result_t<TYPE> m_result;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            async_task_base* m_coroutine_object = nullptr;
            bool m_coroutine_valid = false;
#endif      
        }; // struct promise_type

    protected:
        handle_type m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_t<TYPE> m_result;
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        promise_type* m_promise_type = nullptr;
        bool m_promise_valid = false;
#endif
    }; // template<typename TYPE> class async_task_base


    /**
     * @brief class async_task
     */
    template<typename TYPE>
    class async_task : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;
 
        /** 
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_task(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_task<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
        * @brief this constructor can be used to construct a "placeholder" async_task object
        * to which a "real" async_task object (constructed by the previous constuctor)
        * will be assigned.
        */
        async_task() = default;

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
                    const bool ready = m_async_task.m_coro_handle.promise().m_result.is_ready();
#else
                    const bool ready = m_async_task.m_result.is_ready();
#endif
                    print(PRI2, "%p: async_task<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> h)
                {
                    print(PRI2, "%p: async_task<TYPE>::await_suspend(std::coroutine_handle<> h)\n", this);
                    m_async_task.m_coro_handle.promise().m_continuation = h;
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: async_task<TYPE>::await_resume()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const TYPE r = m_async_task.m_coro_handle.promise().get_result_promise();
#else
                    const TYPE r = m_async_task.m_result.retrieve_result();
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
                return async_task<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_never{};
            }

            struct final_awaiter0 : public final_awaiter_tracker
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter0::await_ready()\n", this);
                    return false;
                }

                void await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter0::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter1::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                    return true;
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
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        return h.promise().m_continuation;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<TYPE>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#else
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter0{};
            }
#endif
        }; // struct promise_type

    }; // template<typename TYPE> class async_task


    /**
     * @class async_ltask
     */
    template<typename TYPE>
    class async_ltask : public async_task_base<TYPE>
    {
    public:
        using handle_type = typename async_task_base<TYPE>::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        /**
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_ltask(handle_type h)
            : async_task_base<TYPE>(h)
        {
            print(PRI2, "%p: async_ltask<TYPE>::async_task(handle_type h)\n", this);
        }

        /**
        * @brief this constructor can be used to construct a "placeholder" async_task object
        * to which a "real" async_task object (constructed by the previous constuctor)
        * will be assigned.
        */
        async_ltask() = default;

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
                    const bool ready = m_async_ltask.m_coro_handle.promise().m_result.is_ready();
#else
                    const bool ready = m_async_ltask.m_result.is_ready();
#endif
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> h)
                {
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_suspend(std::coroutine_handle<> h)\n", this);
                    m_async_ltask.m_coro_handle.promise().m_continuation = h;

                    m_async_ltask.m_coro_handle.resume();
                }

                TYPE await_resume()
                {
                    print(PRI2, "%p: m_async_ltask<TYPE>::await_resume()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                    const TYPE r = m_async_ltask.m_coro_handle.promise().get_result_promise();
#else
                    const TYPE r = m_async_ltask.m_result.retrieve_result();
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
                return async_ltask<TYPE>{handle_type::from_promise(*this)};
            }

            auto initial_suspend()
            {
                print(PRI2, "%p: async_ltask<TYPE>::promise_type::initial_suspend()\n", this);
                return std::suspend_always{};
            }

            struct final_awaiter0 : public final_awaiter_tracker
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter0::await_ready()\n", this);
                    return false;
                }

                void await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter0::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter0::await_resume()\n", this);
                }
            };

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter1::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                    return true;
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
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter2::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this,  h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        return h.promise().m_continuation;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<TYPE>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#else
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter0{};
            }
#endif
        }; // struct promise_type

    }; // template<typename TYPE> class async_ltask

    // ---------------------------------------------------------------------
    // ---------------------------------------------------------------------

    /**
     * @brief class async_task_void
     */
    class async_task_void : public async_base, private coroutine_tracker
    {
    public:

        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        /**
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_task_void(handle_type h)
            : m_coro_handle{ h }
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ }
#endif
        {
            print(PRI2, "%p: async_task_base::async_task_base(handle_type h): promise = %p\n", this, &m_coro_handle.promise());
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_coro_handle.promise().link_coroutine_object(this);
#endif
        }

        /**
         * @brief this constructor can be used to construct a "placeholder" async_task object
         * to which a "real" async_task object (constructed by the previous constuctor)
         * will be assigned.
         */
        async_task_void()
            : m_coro_handle{ nullptr }
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ }
#endif
        {
        }

        async_task_void(const async_task_void&) = delete;
#if DECLARE_MOVE_CONSTRUCTORS_AS_DELETED
        async_task_void(async_task_void&&) = delete;
#else
        async_task_void(async_task_void&& other) noexcept
            : m_coro_handle{ other.m_coro_handle}
#if USE_RESULT_FROM_COROUTINE_OBJECT
            , m_result{ std::move(other.m_result) }
#endif
        {
            print(PRI2, "%p: async_task_void::async_task_void(async_task&& other)\n", this);
            other.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            other.m_result = {};
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = other.m_promise_type;
            m_promise_valid = other.m_promise_valid;
            other.m_promise_type = nullptr;
            other.m_promise_valid = false;
#endif
        }
#endif

        void destroy_coroutine_frame()
        {
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

        ~async_task_void()
        {
            print(PRI2, "%p: async_task_void::~async_task_void()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            coroutine_destructor_admin();
#endif
            destroy_coroutine_frame();
        }

        async_task_void& operator = (const async_task_void&) = delete;
#if DECLARE_MOVE_ASSIGMENT_OPERATORS_AS_DELETED
        async_task_void& operator = (async_task_void&&) = delete;
#else
        async_task_void& operator = (async_task_void&& other)
        {
            print(PRI2, "%p: async_task_void::async_task_void = (async_task&& other)\n", this);
            destroy_coroutine_frame();
            m_coro_handle = other.m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            m_result = std::move(s.m_result);
#endif
            other.m_coro_handle = nullptr;
#if USE_RESULT_FROM_COROUTINE_OBJECT
            other.m_result = {};
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            m_promise_type = other.m_promise_type;
            m_promise_valid = other.m_promise_valid;
            other.m_promise_type = nullptr;
            other.m_promise_valid = false;
#endif
            return *this;
        }
#endif
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
         * @brief waitIfNotReady instructs wait to wait by default
         * if the promise is not ready.
         */
        void wait(bool waitIfNotReady = true)
        {
            print(PRI2, "%p: async_task_void::wait(%d)\n", this, waitIfNotReady);
            if (waitIfNotReady) {
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                print(PRI2, "%p: async_task_base::wait(): m_coro_handle.promise().m_result.wait_for_result()\n", this);
                if (m_coro_handle.promise().m_result.wait_for_result())
#else
                print(PRI2, "%p: async_task_base::wait(): m_result.wait_for_result()\n", this);
                if (m_result.wait_for_result())
#endif
                {
                    print(PRI2, "%p: async_task_base::wait(): m_coro_handle.promise().m_sema.wait()\n", this);
                    m_coro_handle.promise().m_sema.wait();
                }
            }

#if !USE_RESULT_FROM_COROUTINE_OBJECT
            m_coro_handle.promise().m_result.retrieve_result();
#else
            m_result.retrieve_result();
#endif
        }

        bool is_ready() override
        {
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            ready_admin();
#endif
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            print(PRI2, "%p: void async_task_void::is_ready() returns %d\n", this, m_coro_handle.promise().m_result.is_ready());
            return m_coro_handle.promise().m_result.is_ready();
#else
            print(PRI2, "%p: void async_task_void::is_ready() returns %d\n", this, m_result.is_ready());
            return m_result.is_ready();
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
                : m_continuation(nullptr)
                , m_ctr(nullptr)
                , m_waitany(nullptr)
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                , m_result{ }
#endif
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

            /**
             * @brief inform_interested_parties is an auxiliary function that
             * informs the coroutine that references this async_task object using a when_all or a when_any
             * or a function that awaits the completion of this async_task in a different thread
             * that the async_task has completed.
             * The function is called from return_void.
             * See async_task_base<TYPE>::promise_type::inform_interested_parties and
             * async_operation_base::inform_interested_parties for similar functions.
             */
            void inform_interested_parties()
            {
                print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties():\n\tm_ctr = %p, m_waitany = %p, m_continuation = %p\n",
                    this, m_ctr, m_waitany, m_continuation);
                if (m_ctr)
                {
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): before m_ctr->completed();\n", this);
                    m_continuation = m_ctr->completed();
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): after m_ctr->completed();\n", this);
                }
                else if (m_waitany)
                {
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): before m_waitany->completed();\n", this);
                    m_continuation = m_waitany->completed();
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): after m_waitany->completed();\n", this);
                }
                else if (m_result.wait_for_semaphore_release())
                {
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): before m_sema.signal();\n", this);
                    m_sema.signal();
                    print(PRI2, "%p: async_task_void::promise_type::inform_interested_parties(): after m_sema.signal();\n", this);
                }
            }

            void return_void()
            {
                print(PRI2, "%p: async_task_void::promise_type::return_void(): begin\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
                print(PRI2, "%p: async_task_void::promise_type::return_void(): m_coroutine_object = %p (m_coroutine_valid = %d)\n",
                                 this, m_coroutine_object, m_coroutine_valid);
#endif
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_result.set_value(0);
#else
                if (m_coroutine_valid)
                    m_coroutine_object->m_result.set_value(0);
#endif
                inform_interested_parties();
                print(PRI2, "%p: async_task_void::promise_type::return_void(): end\n", this);
            }

            void unhandled_exception()
            {
                print(PRI1, "%p: async_task_void::promise_type::unhandled_exception()\n", this);
#if !USE_RESULT_FROM_COROUTINE_OBJECT
                m_result.set_exception(std::current_exception());
#else
                if (m_coroutine_valid)
                {
                    m_coroutine_object->m_result.set_exception(std::current_exception());
                }
#endif
            }

#if !USE_RESULT_FROM_COROUTINE_OBJECT
            void get_result_promise()
            {
                print(PRI2, "%p: async_task_void::promise_type::get_result_promise()\n", this);
                m_result.retrieve_result();
            }
#endif

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            void link_coroutine_object(async_task_void* coroutine_object)
            {
                m_coroutine_object = coroutine_object;
                m_coroutine_valid = true;
                m_coroutine_object->link_promise_type(this);
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
            std::coroutine_handle<> m_continuation;
            when_all_counter* m_ctr;
            when_any_one* m_waitany;
            Semaphore m_sema;
#if !USE_RESULT_FROM_COROUTINE_OBJECT
            result_t<int> m_result;     // cannot use void
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            async_task_void* m_coroutine_object = nullptr;
            bool m_coroutine_valid = false;
#endif
        }; // struct promise_type

    protected:
        handle_type m_coro_handle;
#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_t<int> m_result;         // cannot use void
#endif
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        promise_type* m_promise_type = nullptr;
        bool m_promise_valid = false;
#endif
    }; // class async_task_void


    /**
     * @brief class async_task<void>
     */
    template<>
    class async_task<void> : public async_task_void
    {
    public:
    
        using handle_type = typename async_task_void::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        /**
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_task(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_task<void>::async_task(handle_type h)\n", this);
        }

        /**
         * @brief this constructor can be used to construct a "placeholder" async_task object
         * to which a "real" async_task object (constructed by the previous constuctor)
         * will be assigned.
         */
        async_task() = default;

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
                    const bool ready = m_async_task.m_coro_handle.promise().m_result.is_ready();
#else
                    const bool ready = m_async_task.is_ready();
#endif
                    print(PRI2, "%p: async_task<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> h)
                {
                    print(PRI2, "%p: async_task<void>::await_suspend(std::coroutine_handle<> h)\n", this);
                    m_async_task.m_coro_handle.promise().m_continuation = h;
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

            struct final_awaiter0 : public final_awaiter_tracker
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter0::await_ready()\n", this);
                    return false;
                }

                void await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter0::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter0::await_resume()\n", this);
                }
            };

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter1::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                    return true;
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
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter2::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        return h.promise().m_continuation;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_task<void>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#else
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter0{};
            }
#endif
        }; // struct promise_type

    }; // template<> class async_task<void>
    

    /**
     * @brief class async_ltask<void>
     */
    template<>
    class async_ltask<void> : public async_task_void
    {
    public:
        using handle_type = typename async_task_void::handle_type;

        struct promise_type;
        using handle_type_own = std::coroutine_handle<promise_type>;

        /**
         * @brief this constructor is called from promise_type::get_return_object()
         */
        async_ltask(handle_type h)
            : async_task_void(h)
        {
            print(PRI2, "%p: async_ltask<void>::async_ltask(handle_type h)\n", this);
        }

        /**
         * @brief this constructor can be used to construct a "placeholder" async_task object
         * to which a "real" async_task object (constructed by the previous constuctor)
         * will be assigned.
         */
        async_ltask() = default;
        
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
                    const bool ready = m_async_ltask.m_coro_handle.promise().m_result.is_ready();
#else
                    const bool ready = m_async_ltask.is_ready();
#endif
                    print(PRI2, "%p: async_ltask<void>::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> h)
                {
                    print(PRI2, "%p: async_ltask<void>::await_suspend(std::coroutine_handle<> h)\n", this);
                    m_async_ltask.m_coro_handle.promise().m_continuation = h;

                    m_async_ltask.m_coro_handle.resume();
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

            struct final_awaiter0 : public final_awaiter_tracker
            {
                bool await_ready() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter0::await_ready()\n", this);
                    return false;
                }

                void await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter0::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter0::await_resume()\n", this);
                }
            };

            struct final_awaiter1 : public final_awaiter_tracker
            {
                bool await_ready() const noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_ready()\n", this);
                    return false;
                }

                bool await_suspend(handle_type_own h) noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter1::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        h.promise().m_continuation.resume();
                    return true;
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
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter2::await_suspend()\n\th.promise().m_continuation = %p\n",
                        this, h.promise().m_continuation);
                    if (h.promise().m_continuation)
                        return h.promise().m_continuation;
                    else
                        return std::noop_coroutine();
                }

                void await_resume() noexcept
                {
                    print(PRI2, "%p: async_ltask<void>::promise_type::final_awaiter2::await_resume()\n", this);
                }
            };

#if USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter1{};
            }
#elif USE_FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_HANDLE
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter2{};
            }
#else
            auto final_suspend() noexcept
            {
                print(PRI2, "%p: async_task<TYPE>::promise_type::final_suspend()\n", this);
                return final_awaiter0{};
            }
#endif
        }; // struct promise_type

    }; // template<> class async_ltask<void> 

} // namespace corolib

#endif
