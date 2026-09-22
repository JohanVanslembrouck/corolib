/**
 * @file result.h
 * @brief Auxiliary classes used in async_task.h.
 * 
 * class result_t is normally used in the definition of promise types.
 * In the alternative implementation (USE_RESULT_FROM_COROUTINE_OBJECT = 1),
 * result_t is used the definition of async_task types.
 *
 * @author Johan Vanslembrouck
 */

 /**
 Class hierarchy
 ===============

                          class result_base_t
                                    ^
                                    |
                                    |
                -----------------------------------------
                |                                       |
                |                                       |
        template<typename TYPE>                     template<>
        class result_t                              class result_t<void>

 */
 
#ifndef _RESULT_H_
#define _RESULT_H_

#if USE_IN_MT_APPS
#include <atomic>
#endif

#include "print.h"

namespace corolib
{
    // ---------------------------------------------------------------------
    // class result_base_t
    // ---------------------------------------------------------------------

    class result_base_t
    {
    protected:
        enum class completion_status : char
        {
            INITIAL = 0,
            COMPLETED,
            WAIT_FOR_SEMAPHORE_RELEASE,
        };

    public:
        result_base_t()
            : m_exception{ nullptr }
            , m_ready{ completion_status::INITIAL }
            , m_wait_for_semaphore_release{ false }
        {
            clprint(PRI2, "%p: result_base_t::result_base_t();\n", this);
        }

        ~result_base_t()
        {
            clprint(PRI2, "%p: result_base_t::~result_base_t();\n", this);

            m_exception = nullptr;
            m_ready = completion_status::INITIAL;
            m_wait_for_semaphore_release = false;
        }

        result_base_t(const result_base_t&) = delete;

#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_base_t(result_base_t&& other)
            : m_exception{ other.m_exception }
#if !USE_IN_MT_APPS
            , m_ready{ other.m_ready }
#endif
            , m_wait_for_semaphore_release{ other.m_wait_for_semaphore_release }
        {
            clprint(PRI2, "%p: result_base_t::result_base_t(result_base_t&& other);\n", this);
#if USE_IN_MT_APPS
            m_ready.store(other.m_ready.load());
#endif
            other.m_exception = nullptr;
            other.m_ready = completion_status::INITIAL;
            other.m_wait_for_semaphore_release = false;
        }
#else
        result_base_t(result_base_t&& other) = delete;
#endif

        result_base_t& operator = (const result_base_t&) = delete;

#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_base_t& operator = (result_base_t&& other) noexcept
        {
            clprint(PRI2, "%p: result_base_t::operator = (result_base_t&& other);\n", this);
            m_exception = other.m_exception;
#if USE_IN_MT_APPS
            m_ready.store(other.m_ready.load());
#else
            m_ready = other.m_ready;
#endif
            m_wait_for_semaphore_release = other.m_wait_for_semaphore_release;
            other.m_exception = nullptr;
            other.m_ready = completion_status::INITIAL;
            other.m_wait_for_semaphore_release = false;
            return *this;
        }
#else
        result_base_t& operator = (result_base_t&& other) noexcept = delete;
#endif

        // set_value() is called from promise_task_type<void>::return_void()
        // and as an auxiliary function also from result_t<TYPE>::set_value(const TYPE& value)
        void set_value()
        {
            clprint(PRI2, "%p: result_base_t::set_value();\n", this);

#if USE_IN_MT_APPS
            completion_status expected = completion_status::INITIAL;
            if (!m_ready.compare_exchange_strong(expected, completion_status::COMPLETED)) {
                if (expected != completion_status::WAIT_FOR_SEMAPHORE_RELEASE)
                    clprint(PRI1, "%p: result_base_t::set_value(): expected = %d != completion_status::WAIT_FOR_SEMAPHORE_RELEASE\n", this, static_cast<char>(expected));
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

        // set_exception(...) is called from promise_task_type<TYPE>::unhandled_exception()
        // and from promise_task_type<void>::unhandled_exception().
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
            clprint(PRI2, "%p: result_base_t::is_ready(): return %d;\n", this, ready);
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
                    clprint(PRI1, "%p: result_base_t::wait_for_result(): expected = %d != completion_status::COMPLETED\n", this, static_cast<char>(expected));
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
            clprint(PRI2, "%p: result_base_t::wait_for_result(): return %d;\n", this, wait);
            return wait;
        }

        bool wait_for_semaphore_release()
        {
            clprint(PRI2, "%p: result_base_t::wait_for_semaphore_release(): return %d;\n", this, m_wait_for_semaphore_release);
            return m_wait_for_semaphore_release;
        }

    protected:
        std::exception_ptr m_exception;
#if USE_IN_MT_APPS
        std::atomic<completion_status> m_ready{ completion_status::INITIAL };
#else
        completion_status m_ready{ completion_status::INITIAL };
#endif
        // m_wait_for_semaphore_release is accessed from a single thread only (the completion thread).
        // Therefore it does not have to be atomic.
        bool m_wait_for_semaphore_release{ false };
    };

    // ---------------------------------------------------------------------
    // template<typename TYPE> class result_t
    // ---------------------------------------------------------------------

    /**
     * @brief class result_t
     */
    template<typename TYPE>
    class result_t : public result_base_t
    {
    public:
        result_t()
            : result_base_t{}
            , m_value{}
        {
            clprint(PRI2, "%p: result_t<TYPE>::result_t();\n", this);
        }

        ~result_t()
        {
            clprint(PRI2, "%p: result_t<TYPE>::~result_t();\n", this);
            m_value = {};
        }

        result_t(const result_t&) = delete;

#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_t(result_t&& other)
            : result_base_t{ std::move(other) }
            , m_value{ other.value }
        {
            clprint(PRI2, "%p: result_t<TYPE>::result_t(result_t&& other);\n", this);
        }
#else
        result_t(result_t&& other) = delete;
#endif

        result_t& operator = (const result_t&) = delete;

#if USE_RESULT_FROM_COROUTINE_OBJECT
        result_t& operator = (result_t&& other) noexcept
        {
            clprint(PRI2, "%p: result_t<TYPE>::operator = (result_t&& other);\n", this);
            result_base_t::operator=(std::move(other));
            m_value = other.m_value;
            return *this;
        }
#else
        result_t& operator = (result_t&& other) noexcept = delete;
#endif

        // set_value(const TYPE& value) is called from promise_task_type<TYPE>::return_value(TYPE v)
        void set_value(const TYPE& value)
        {
            clprint(PRI2, "%p: result_t<TYPE>::set_value(const TYPE& value);\n", this);
            m_value = value;
            result_base_t::set_value();
        }

        TYPE retrieve_result()
        {
            if (m_ready == completion_status::INITIAL)
                clprint(PRI1, "%p: result_t<TYPE>::retrieve_result(): m_ready == INITIAL!!!\n", this);
            if (m_exception != nullptr)
            {
                clprint(PRI1, "%p: result_t<TYPE>::retrieve_result(): std::rethrow_exception(m_exception);\n", this);
                std::rethrow_exception(m_exception);
            }
            clprint(PRI2, "%p: result_t<TYPE>::retrieve_result(): return m_value;\n", this);
            return m_value;
        }

    private:
        TYPE m_value{};
    };

    // ---------------------------------------------------------------------
    // template<> class result_t<void>
    // ---------------------------------------------------------------------

    template<>
    class result_t<void> : public result_base_t
    {
    public:
        void retrieve_result()
        {
            if (m_ready == completion_status::INITIAL)
                clprint(PRI1, "%p: result_t<void>::retrieve_result(): m_ready == INITIAL!!!\n", this);
            if (m_exception != nullptr)
            {
                clprint(PRI1, "%p: result_t<void>::retrieve_result(): std::rethrow_exception(m_exception);\n", this);
                std::rethrow_exception(m_exception);
            }
            clprint(PRI2, "%p: result_t<void>::retrieve_result(): return;\n", this);
        }
    };
}

#endif
