/**
 * @file when_all.h
 * @brief
 * when_all waits for all async_operations or async_task objects passed to it in its constructor
 * to complete.
 *
 * when_all passes its m_counter data member object to every async_operation or async_task object.
 * 
 * When an async_operation or async_task completes, it decrements the counter in the m_counter object.
 * When that counter reaches 0, the coroutines co_awaiting the when_all object will be resumed.
 *
 * TODO: verify instantiation of when_all with an appropriate type using C++20 concepts.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ALL
#define _WHEN_ALL

#include <vector>
#include <type_traits>

#include "print.h"
#include "when_all_counter.h"
#include "async_base.h"

namespace corolib
{
    class when_all
    {
    public:

        /**
         * @brief constructor that takes a variable list of async_base-derived objects and
         * populates the internal vector m_elements with its elements.
         */
        template<typename... AsyncBaseTypes>
        when_all(AsyncBaseTypes&... others)
            : m_counter(0)
        {
            print(PRI2, "%p: when_all::when_all(AsyncBaseTypes&... others)\n", this);
            int len = make_when_all_len(0, others...);
            print(PRI2, "%p: when_all::when_all(AsyncBaseTypes&... others): len = %d\n", this, len);
            m_elements.reserve(len);
            make_when_all(others...);
        }

        /**
         * @brief constructor that takes an initializer list and
         * populates the internal vector m_elements with its elements.
         */
        when_all(std::initializer_list<async_base*> async_ops)
            : m_counter(0)
        {
            print(PRI2, "%p: when_all::when_all(std::initializer_list<async_base*> async_ops)\n", this);
            m_elements.reserve(async_ops.size());

            for (async_base* async_op : async_ops)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!async_op->is_ready())
                {
                    async_op->setCounter(&m_counter);
                    m_elements.push_back(async_op);
                    m_counter.increment();
                }
            }
        }

        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
         * and that populates the internal vector m_elements with its elements.
         */
        when_all(async_base* pasync_ops[], int size)
            : m_counter(0)
        {
            print(PRI2, "%p: when_all::when_all(async_base* pasync_ops, size = %d)\n", this, size);
            m_elements.reserve(size);

            for (int i = 0; i < size; i++)
            {
                // Only place the object in m_elements if it has not yet been completed.
                async_base* op = pasync_ops[i];
                print(PRI2, "%p: when_all::when_all: op = %p\n", this, op);
                if (!op->is_ready())
                {
                    op->setCounter(&m_counter);
                    m_elements.push_back(op);
                    m_counter.increment();
                }
            }
        }

        when_all(const when_all& s) = delete;
        when_all(when_all&& s) = delete;

        ~when_all()
        {
            print(PRI2, "%p: when_all::~when_all()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->setCounter(nullptr);
            }
        }

        when_all& operator = (const when_all&) = delete;
        when_all& operator = (when_all&& s) = delete;

        /**
         * @brief
         *
         */
        void start_all()
        {
            print(PRI2, "%p: when_all:start_all()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->start();
            }
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_all& when_all_)
                    : m_when_all(when_all_)
                {
                    print(PRI2, "%p: when_all::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    print(PRI2, "%p: when_all::awaiter::await_ready(): m_when_all.m_counter.get_counter() = %d;\n",
                        this, m_when_all.m_counter.get_counter());
                    bool ready = (m_when_all.m_counter.get_counter() == 0);
                    print(PRI2, "%p: when_all::awaiter::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: when_all::awaiter::await_suspend(...)\n", this);
                    m_when_all.start_all();    // Will have no effect in case of an eager start
                    m_when_all.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    print(PRI2, "%p: when_all::awaiter::await_resume()\n", this);
                }
            private:
                when_all& m_when_all;
            };

            return awaiter{ *this };
        }

    protected:

        // ---------------------------------------------------------

        template<typename... AsyncBaseTypes>
        int make_when_all_len(int len, AsyncBaseTypes&... others);

        template<typename T, typename... AsyncBaseTypes,
            typename std::enable_if<std::is_base_of_v<async_base, T>, int>::type = 0>
        int make_when_all_len(int len, T& t, AsyncBaseTypes&... others) {
            return make_when_all_len(len + 1, others...);
        };

        //template<>      // g++:  error: explicit specialization in non-namespace scope ‘class corolib::when_any’
        int make_when_all_len(int len) {
            return len;
        };

        // ---------------------------------------------------------

        template<typename... AsyncBaseTypes>
        void make_when_all(AsyncBaseTypes&... others);

        template<typename T, typename... AsyncBaseTypes, 
                 typename std::enable_if<std::is_base_of_v<async_base, T>, int>::type = 0>
        void make_when_all(T& t, AsyncBaseTypes&... others) {
            async_base* async_op = static_cast<async_base*>(&t);
            print(PRI2, "%p: make_when_all()\n", this);
            // Only place the object in m_elements if it has not yet been completed.
            if (!async_op->is_ready())
            {
                async_op->setCounter(&m_counter);
                m_elements.push_back(async_op);
                m_counter.increment();
            }
            make_when_all(others...);
        };

        //template<>      // g++: error: explicit specialization in non-namespace scope ‘class corolib::when_all’
        void make_when_all() {
        };

        // ---------------------------------------------------------

    private:
        when_all_counter m_counter;
        std::vector<async_base*> m_elements;
    };

    /**
     * @brief when_allT is an alternative to when_all.
     * It has only one constructor when_allT(TYPE aws[], int size) that allows passing a C-style array of TYPE objects,
     * together with its size.
     * This avoids the use of an auxiliary array of async_base* to be used with when_all(async_base* pasync_ops[], int size).
     * TYPE must be an async_operation or an async_task
     */
    template<typename TYPE>
    class when_allT
    {
    public:
        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
		 * and that populates the internal vector m_elements with its elements.
         */
        when_allT(TYPE aws[], int size)
            : m_counter(0)
        {
            print(PRI2, "%p: when_allT::when_allT(TYPE* aws, size = %d)\n", this, size);
            m_elements.reserve(size);

            for (int i = 0; i < size; i++)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!aws[i].is_ready())
                {
                    aws[i].setCounter(&m_counter);
                    m_elements.push_back(&aws[i]);
                    m_counter.increment();
                }
            }
        }

        when_allT(const when_allT& s) = delete;

        when_allT(when_allT&& s)
        {
            print(PRI2, "%p: when_allT::when_allT(when_all&& s)\n", this);
        }

        ~when_allT()
        {
            print(PRI2, "%p: when_allT::~when_allT()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->setCounter(nullptr);
            }
        }

        when_allT& operator = (const when_allT&) = delete;

        when_allT& operator = (when_allT&& s)
        {
            print(PRI2, "%p: when_allT::when_allT = (when_all&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        /**
         * @brief
         *
         */
        void start_all()
        {
            print(PRI2, "%p: when_allT:start_all()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->start();
            }
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_allT& when_all_)
                    : m_when_all(when_all_)
                {
                    print(PRI2, "%p: when_allT::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    print(PRI2, "%p: when_allT::awaiter::await_ready(): m_when_all.m_counter.get_counter() = %d;\n", 
                            this, m_when_all.m_counter.get_counter());
                    bool ready = (m_when_all.m_counter.get_counter() == 0);
                    print(PRI2, "%p: when_allT::awaiter::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: when_allT::awaiter::await_suspend(...)\n", this);
                    m_when_all.start_all();    // Will have no effect in case of an eager start
                    m_when_all.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    print(PRI2, "%p: when_allT::awaiter::await_resume()\n", this);
                }
            private:
                when_allT& m_when_all;
            };

            return awaiter{ *this };
        }

    private:
        when_all_counter m_counter;
        std::vector<TYPE*> m_elements;
    };

}

#endif
