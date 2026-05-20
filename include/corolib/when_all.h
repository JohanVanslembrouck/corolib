/**
 * @file when_all.h
 * @brief
 * when_all waits for all async_operation or async_task or async_ltask objects passed to it in its constructor
 * to complete.
 *
 * when_all passes its m_counter data member object to every async_operation or async_task object.
 * 
 * When an async_operation or async_task or async_ltask completes, it decrements the counter in the m_counter object.
 * When that counter reaches 0, the coroutines co_awaiting the when_all object will be resumed.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _WHEN_ALL_H_
#define _WHEN_ALL_H_

#include <memory>
#include <vector>
#include <array>

#include "print.h"
#include "when_all_counter.h"

#include "config.h"

#if !USE_WHEN_TYPE
#include "async_base.h"
#else
#include "when_type.h"
#endif

namespace corolib
{
#if USE_WHEN_TYPE
    class when_all_info_base
    {
    public:
        virtual bool is_ready() = 0;
        virtual void setCounter(when_all_counter* ctr) = 0;
        virtual void setWaitAny(when_any_one* waitany) = 0;
        virtual void start() = 0;
    };

    template<WHEN_TYPE when_type>
    class when_all_info : public when_all_info_base
    {
    public:
        when_all_info(when_type& task)
            : m_task(task)
        {
            clprint(PRI2, "when_all_info::when_all_info(...)\n");
        }

        ~when_all_info()
        {
            clprint(PRI2, "when_all_info::~when_all_info()\n");
        }

        bool is_ready() override
        {
            return m_task.is_ready();
        }

        void setCounter(when_all_counter* ctr) override
        {
            m_task.setCounter(ctr);
        }

        void setWaitAny(when_any_one* waitany) override
        {
            m_task.setWaitAny(waitany);
        }

        void start() override
        {
            m_task.start();
        }

    private:
        when_type& m_task;
    };
#endif

    class when_all
    {
    public:

#if !USE_WHEN_TYPE
        /**
         * @brief constructor that takes an initializer list and
         * populates the internal vector m_elements with its elements.
         * This constructor is deprecated:
         * please use when_all(AsyncBaseTypes&... others) instead.
         */
        [[deprecated]]
        when_all(std::initializer_list<async_base*> async_ops)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all::when_all(std::initializer_list<async_base*> async_ops)\n", this);
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
         * This constructor is deprecated:
         * please use when_all(std::array<AsyncBaseType, Size>& async_ops) or
         *            when_all(std::vector<AsyncBaseType>& async_ops) instead.
         */
        [[deprecated]]
        when_all(async_base* pasync_ops[], int size)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all::when_all(async_base* pasync_ops, size = %d)\n", this, size);
            m_elements.reserve(size);

            for (int i = 0; i < size; i++)
            {
                // Only place the object in m_elements if it has not yet been completed.
                async_base* op = pasync_ops[i];
                clprint(PRI2, "%p: when_all::when_all: op = %p\n", this, op);
                if (!op->is_ready())
                {
                    op->setCounter(&m_counter);
                    m_elements.push_back(op);
                    m_counter.increment();
                }
            }
        }

        template<typename AsyncBaseType, int Size>
        when_all(std::array<AsyncBaseType, Size>& async_ops)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all(std::array<AsyncBaseType, Size>)\n", this);
            size_t len = async_ops.size();

            int i = 0;
            for (async_base& async_op : async_ops)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!async_op.is_ready())
                {
                    async_op.setCounter(&m_counter);
                    m_elements.push_back(&async_ops[i]);
                    m_counter.increment();
                }
                i++;
            }
        }

        template<typename AsyncBaseType>
        when_all(std::vector<AsyncBaseType>& async_ops)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all(std::vector<AsyncBaseType>)\n", this);
            size_t len = async_ops.size();
            m_elements.reserve(len);

            int i = 0;
            for (async_base& async_op : async_ops)
            {
                // Only place the object in m_elements if it has not yet been completed.
                if (!async_op.is_ready())
                {
                    async_op.setCounter(&m_counter);
                    m_elements.push_back(&async_ops[i]);
                    m_counter.increment();
                }
                i++;
            }
        }
#else
        template<typename AsyncBaseType, int Size>
        when_all(std::array<AsyncBaseType, Size>& async_ops)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all(std::array<AsyncBaseType>, Size)\n", this);
            size_t len = async_ops.size();
            clprint(PRI2, "%p, when_all::when_all(std::array<AsyncBaseType, Size>): len = %zd\n", this, len);
            m_elements.reserve(len);

            for (AsyncBaseType& async_op : async_ops)
            {
                make_when_allV(async_op);
            }
        }

        template<typename AsyncBaseType>
        when_all(std::vector<AsyncBaseType>& async_ops)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all(std::vector<AsyncBaseTypes>)\n", this);
            size_t len = async_ops.size();
            clprint(PRI2, "when_all::when_all(AsyncBaseTypes&... others): len = %zd\n", len);
            m_elements.reserve(len);

            for (AsyncBaseType& async_op : async_ops)
            {
                make_when_allV(async_op);
            }
        }
#endif

        /**
        * @brief constructor that takes a variable list of async_base-derived objects and
        * populates the internal vector m_elements with its elements.
        * 
        * Note: this definition has to be placed behind the definitions for std::array and std::vector;
        * otherwise, gcc starts expanding this definition with a std::array or std::vector
        * and then runs into problems because, in make_when_all, std::array and std::vector
        * do not satisfy the WHEN_TYPE requirements.
        */
        template<typename... AsyncBaseTypes>
        when_all(AsyncBaseTypes&... others)
            : m_counter(0)
        {
            clprint(PRI2, "%p: when_all::when_all(AsyncBaseTypes&... others)\n", this);
            int len = sizeof... (others);
            clprint(PRI2, "%p: when_all::when_all(AsyncBaseTypes&... others): len = %d\n", this, len);
            m_elements.reserve(len);
            make_when_all(others...);
        }

        when_all(const when_all& s) = delete;
        when_all(when_all&& s) = delete;

        ~when_all()
        {
            clprint(PRI2, "%p: when_all::~when_all()\n", this);
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
            clprint(PRI2, "%p: when_all:start_all()\n", this);
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
                    clprint(PRI2, "%p: when_all::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    clprint(PRI2, "%p: when_all::awaiter::await_ready(): m_when_all.m_counter.get_counter() = %d;\n",
                        this, m_when_all.m_counter.get_counter());
                    bool ready = (m_when_all.m_counter.get_counter() == 0);
                    clprint(PRI2, "%p: when_all::awaiter::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    clprint(PRI2, "%p: when_all::awaiter::await_suspend(...)\n", this);
                    m_when_all.start_all();    // Will have no effect in case of an eager start
                    m_when_all.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    clprint(PRI2, "%p: when_all::awaiter::await_resume()\n", this);
                }
            private:
                when_all& m_when_all;
            };

            return awaiter{ *this };
        }

    protected:

        template<typename... AsyncBaseTypes>
        void make_when_all(AsyncBaseTypes&... others);

#if !USE_WHEN_TYPE
        template<typename T, typename... AsyncBaseTypes, 
                 typename std::enable_if<std::is_base_of_v<async_base, T>, int>::type = 0>
        void make_when_all(T& t, AsyncBaseTypes&... others) {
            async_base* async_op = static_cast<async_base*>(&t);
            clprint(PRI2, "%p: make_when_all()\n", this);
            // Only place the object in m_elements if it has not yet been completed.
            if (!async_op->is_ready())
            {
                async_op->setCounter(&m_counter);
                m_elements.push_back(async_op);
                m_counter.increment();
            }
            make_when_all(others...);
        };
#else
        template<WHEN_TYPE when_type, typename... AsyncBaseTypes>
        void make_when_all(when_type& t, AsyncBaseTypes&... others) {
            clprint(PRI2, "%p: make_when_all(T& t, AsyncBaseTypes&... others)\n", this);
            // Only place the object in m_elements if it has not yet been completed.
            if (!t.is_ready())
            {
                t.setCounter(&m_counter);
                std::shared_ptr<when_all_info<decltype(t)>> t1_wr = std::make_shared <when_all_info<decltype(t)>>(t);
                m_elements.push_back(t1_wr);
                m_counter.increment();
            }
            make_when_all(others...);
        };
#endif

        //template<>      // g++: error: explicit specialization in non-namespace scope ‘class corolib::when_all’
        void make_when_all() {
        };

#if USE_WHEN_TYPE
        template<WHEN_TYPE when_type>
        void make_when_allV(when_type& t)
        {
            clprint(PRI2, "%p: make_when_allV)\n", this);
            if (!t.is_ready())
            {
                t.setCounter(&m_counter);
                std::shared_ptr<when_all_info<decltype(t)>> t1_wr = std::make_shared<when_all_info<decltype(t)>>(t);
                m_elements.push_back(t1_wr);
                m_counter.increment();
            }
        }
#endif

    private:
        when_all_counter m_counter;
#if !USE_WHEN_TYPE
        std::vector<async_base*> m_elements;
#else
        std::vector<std::shared_ptr<when_all_info_base>> m_elements;
#endif

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
            clprint(PRI2, "%p: when_allT::when_allT(TYPE* aws, size = %d)\n", this, size);
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
            clprint(PRI2, "%p: when_allT::when_allT(when_all&& s)\n", this);
        }

        ~when_allT()
        {
            clprint(PRI2, "%p: when_allT::~when_allT()\n", this);
            for (std::size_t i = 0; i < m_elements.size(); i++)
            {
                m_elements[i]->setCounter(nullptr);
            }
        }

        when_allT& operator = (const when_allT&) = delete;

        when_allT& operator = (when_allT&& s)
        {
            clprint(PRI2, "%p: when_allT::when_allT = (when_all&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        /**
         * @brief
         *
         */
        void start_all()
        {
            clprint(PRI2, "%p: when_allT:start_all()\n", this);
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
                    clprint(PRI2, "%p: when_allT::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    clprint(PRI2, "%p: when_allT::awaiter::await_ready(): m_when_all.m_counter.get_counter() = %d;\n", 
                            this, m_when_all.m_counter.get_counter());
                    bool ready = (m_when_all.m_counter.get_counter() == 0);
                    clprint(PRI2, "%p: when_allT::awaiter::await_ready(): return %d;\n", this, ready);
                    return ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    clprint(PRI2, "%p: when_allT::awaiter::await_suspend(...)\n", this);
                    m_when_all.start_all();    // Will have no effect in case of an eager start
                    m_when_all.m_counter.set_awaiting(awaiting);
                }

                void await_resume()
                {
                    clprint(PRI2, "%p: when_allT::awaiter::await_resume()\n", this);
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
