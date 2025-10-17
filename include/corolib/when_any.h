/**
 * @file when_any.h
 * @brief
 * when_any waits for any of the async_operation or async_task or async_ltask objects (passed to it in its constructor)
 * to complete.
 * 
 * when_any uses a type when_any_info with two fields:
 *   struct when_any_info
 *   {
 *      async_base* m_element = nullptr;
 *      when_any_one m_when_any_one;
 *   };
 * 
 * async_base is the base class for async_operation and async_task and async_ltask.
 *
 * when_any has the following data member:
 *   std::vector<when_any_info> m_when_any_info_vector;
 * 
 * The vector contains a when_any_info item for every async_operation / async_task / async_ltask object.
 * 
 * The m_element field points to a async_operation / async_task / async_ltask object.
 * 
 * Using setWaitAny(), when_any passes a pointer to a m_when_any_one object to each
 * async_operation / async_task / async_ltask object.
 *
 * Important: to avoid that the location of the m_when_any_one objects changes when new elements are added
 * to the vector via push_back, sufficient space for the m_when_any_info_vector is allocated using reserve().
 * 
 * This way, the async_operation / async_task / async_ltask object can inform when_any via m_when_any_one that it has completed.
 *
 * TODO: verify instantiation of when_any with an appropriate type using C++20 concepts.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _WHEN_ANY_AWAITABLE_
#define _WHEN_ANY_AWAITABLE_

#include <vector>
#include <type_traits>

#include "print.h"
#include "when_any_one.h"
#include "async_base.h"

namespace corolib
{
    struct when_any_info
    {
        async_base* m_element = nullptr;
        when_any_one m_when_any_one;
        
        when_any_info(async_base* element = nullptr)
            : m_element(element)
        {
            clprint(PRI2, "%p: when_any_info::when_any_info(element = %p)\n", this, element);
        }

        when_any_info(const when_any_info& other)
            : m_element(other.m_element)
        {
            clprint(PRI2, "%p: when_any_info::when_any_info(&other = %p)\n", this, &other);
        }

        ~when_any_info()
        {
            clprint(PRI2, "%p: when_any_info::~when_any_info()\n", this, m_element);
            m_element = nullptr;
        }

        when_any_info(when_any_info&&) noexcept = default;

        when_any_info& operator = (const when_any_info&) = delete;
        when_any_info& operator = (when_any_info&&) noexcept = default;
    };

    class when_any
    {
    public:
        /**
         * @brief constructor that takes a variable list of async_base-derived objects and
         * populates the internal vector m_elements with its elements.
         */
        template<typename... AsyncBaseTypes>
        when_any(AsyncBaseTypes&... others)
        {
            clprint(PRI2, "%p: when_any::make_when_any(AsyncBaseTypes&... others)\n", this);
            int len = sizeof... (others);
            clprint(PRI2, "%p: when_any::make_when_any(AsyncBaseTypes&... others): len = %d\n", this, len);
            m_when_any_info_vector.reserve(len);
            make_when_any(0, others...);
        }

        /**
         * @brief constructor that takes an initializer list and
         * populates the internal vector m_elements with its elements.
         */
        when_any(std::initializer_list<async_base*> async_ops)
        {
            clprint(PRI2, "%p: when_any::when_any(std::initializer_list<async_base*> async_ops)\n", this);
            m_when_any_info_vector.reserve(async_ops.size());

            int i = 0;
            for (async_base* async_op : async_ops)
            {
                when_any_info info(async_op);
                m_when_any_info_vector.push_back(info);
                async_op->setWaitAny(&m_when_any_info_vector[i].m_when_any_one);
                // Retrieve status from the async_op and save it
                bool ready = async_op->is_ready();
                m_when_any_info_vector[i].m_when_any_one.set_completed(ready);
                i++;
            }
        }

        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
         * and that populates the internal vector m_elements with its elements.
         */
        when_any(async_base* pasync_ops[], int size)
        {
            clprint(PRI2, "%p: when_any::when_any(async_base* pasync_ops, int size)\n", this);
            m_when_any_info_vector.reserve(size);
            
            for (int i = 0; i < size; i++)
            {
                async_base* async_op = pasync_ops[i];
                when_any_info info(async_op);
                m_when_any_info_vector.push_back(info);
                async_op->setWaitAny(&m_when_any_info_vector[i].m_when_any_one);
                // Retrieve status from the async_op and save it
                bool ready = async_op->is_ready();
                m_when_any_info_vector[i].m_when_any_one.set_completed(ready);
            }
        }

        when_any(const when_any& s) = delete;
        when_any(when_any&& s) noexcept = delete;

        ~when_any()
        {
            clprint(PRI2, "%p: when_any::~when_any()\n", this);
            for (std::size_t i = 0; i < m_when_any_info_vector.size(); i++)
            {
                async_base* async_op = m_when_any_info_vector[i].m_element;
                clprint(PRI2, "%p: when_any::~when_any(): i = %d, async_op = %p\n", this, i, async_op);
                if (async_op)
                    async_op->setWaitAny(nullptr);
                m_when_any_info_vector[i].m_element = nullptr;
            }
            m_when_any_info_vector.clear();
        }

        when_any& operator = (const when_any&) = delete;
        when_any& operator = (when_any&& s) = delete;

        /**
         * @brief start_all starts all lazy start coroutines in m_elements.
         * This should be done only once.
         * An application may/will call co_await several times on a wait_any object,
         * which leads to await_ready being called several times as well.
         * However, the lazy start coroutines should only be started once,
         * i.e. on the first co_await call.
         * To avoid multiple calls of start(), data member m_first is used.
         * This data member is added to when_any instead of to awaiter,
         * because an awaiter object is created for every co_await call.
         */
        void start_all()
        {
            clprint(PRI2, "%p: when_any:start_all(): m_first = %d\n", this, m_first);
            if (m_first)
            {
                for (std::size_t i = 0; i < m_when_any_info_vector.size(); i++)
                {
                    m_when_any_info_vector[i].m_element->start();
                }
                m_first = false;
            }
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_any& when_any_)
                    : m_when_any(when_any_)
                {
                    clprint(PRI2, "%p: when_any::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    clprint(PRI2, "%p: when_any::awaiter::await_ready(): size = %d\n",
                            this, m_when_any.m_when_any_info_vector.size());
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); i++)
                    {
                        if (m_when_any.m_when_any_info_vector[i].m_when_any_one.get_completed())
                        {
                            clprint(PRI2, "%p: when_any::awaiter::await_ready(): return true for i = %d;\n", this, i);
                            return true;
                        }
                    }
                    clprint(PRI2, "%p: when_any::awaiter::await_ready(): return false;\n", this);
                    return false;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    clprint(PRI2, "%p: when_any::awaiter::await_suspend(...): size = %ld\n",
                            this, m_when_any.m_when_any_info_vector.size());
                    m_when_any.start_all();    // Will have no effect in case of an eager start
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); ++i)
                    {
                        when_any_one * p = &m_when_any.m_when_any_info_vector[i].m_when_any_one;
                        clprint(PRI2, "%p: when_any::awaiter::await_suspend(...): p = %p\n", this, p);
                        p->set_awaiting(awaiting);
                    }
                }

                int await_resume()
                {
                    // Find out which one has completed
                    clprint(PRI2, "%p: when_any::awaiter::await_resume(): size = %ld\n",
                            this, m_when_any.m_when_any_info_vector.size());

                    int ret = -1;
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); i++)
                    {
                        if (m_when_any.m_when_any_info_vector[i].m_when_any_one.get_and_mark_as_completed())
                        {
                            clprint(PRI2, "%p: when_any::awaiter::await_resume(): return i = %d\n", this, i);
                            ret = i;
                            break;
                        }
                    }

                    if (ret == -1)
                        clprint(PRI1, "%p: when_any::awaiter::await_resume(): Error, none has completed, yet the coroutine has been resumed\n", this);
                    return ret;
                }
            private:
                when_any& m_when_any;
            };

            return awaiter{ *this };
        }

    protected:

        template<typename... AsyncBaseTypes>
        void make_when_any(int i, AsyncBaseTypes&... others);

        template<typename T, typename... AsyncBaseTypes,
                 typename std::enable_if<std::is_base_of_v<async_base, T>, int>::type = 0>
        void make_when_any(int i, T& t, AsyncBaseTypes&... others) {
            clprint(PRI2, "%p: make_when_any() - begin\n", this);

            async_base* async_op = static_cast<async_base*>(&t);
            when_any_info info(async_op);
            m_when_any_info_vector.push_back(info);
            async_op->setWaitAny(&m_when_any_info_vector[i].m_when_any_one);
            // Retrieve status from the async_op and save it
            bool ready = async_op->is_ready();
            m_when_any_info_vector[i].m_when_any_one.set_completed(ready);

            make_when_any(i + 1, others...);
            clprint(PRI2, "%p: make_when_any() - end\n", this);
        };

        //template<>      // g++:  error: explicit specialization in non-namespace scope ‘class corolib::when_any’
        void make_when_any(int) {
        };

    private:
        std::vector<when_any_info> m_when_any_info_vector;
        bool m_first{ true };
    };



    template<typename TYPE>
    struct when_any_infoT
    {   
        TYPE* m_element = nullptr;
        when_any_one m_when_any_one;
    };
    
    /**
    * @brief when_anyT is an alternative to when_any.
    * It has only one constructor when_anyT(TYPE aws[], int size) that allows passing a C-style array of TYPE objects,
    * together with its size.
    * This avoids the use of an auxiliary array of async_base* to be used with when_any(async_base* pasync_ops[], int size).
    * TYPE must be an async_operation or an async_task or an async_ltask object.
    */
    template<typename TYPE>
    class when_anyT
    {
    public:
        /**
         * @brief constructor that takes a pointer to a C-style array of objects and its size
         * and that populates the internal vector m_elements with its elements.
         */
        when_anyT(TYPE aws[], int size)
        {
            clprint(PRI2, "%p: when_anyT::when_anyT(TYPE aws[], int size)\n", this);
            m_when_any_info_vector.reserve(size);

            for (int i = 0; i < size; i++)
            {
                when_any_infoT<TYPE> q(&aws[i]);
                m_when_any_info_vector.push_back(q);
                aws[i].setWaitAny(&m_when_any_info_vector[i].m_when_any_one);
                // Retrieve status from the async_op and save it
                bool ready = aws[i].is_ready();
                m_when_any_info_vector[i].m_when_any_one.set_completed(ready);
            }
        }

        when_anyT(const when_any& s) = delete;

        when_anyT(when_any&& s)
        {
            clprint(PRI2, "%p: when_anyT::when_anyT(when_any&& s)\n", this);
        }

        ~when_anyT()
        {
            clprint(PRI2, "%p: when_anyT::~when_anyT()\n", this);
            for (std::size_t i = 0; i < m_when_any_info_vector.size(); i++)
            {
                async_base* async_op = m_when_any_info_vector[i].m_element;
                clprint(PRI2, "%p: when_anyT::~when_anyT(): i = %d, async_op = %p\n", this, i, async_op);
                if (async_op)
                    async_op->setWaitAny(nullptr);
                m_when_any_info_vector[i].m_element = nullptr;
            }
            m_when_any_info_vector.clear();
        }

        when_anyT& operator = (const when_anyT&) = delete;

        when_anyT& operator = (when_anyT&& s)
        {
            clprint(PRI2, "%p: when_anyT::operator = (when_anyT&& s)\n", this);
            s.coro = nullptr;
            return *this;
        }

        /**
         * @brief start_all starts all lazy start coroutines in m_elements.
         * This should be done only once.
         * An application may/will call co_await several times on a wait_any object,
         * which leads to await_ready being called several times as well.
         * However, the lazy start coroutines should only be started once,
         * i.e. on the first co_await call.
         * To avoid multiple calls of start(), data member m_first is used.
         * This data member is added to when_any instead of to awaiter,
         * because an awaiter object is created for every co_await call.
         */
        void start_all()
        {
            clprint(PRI2, "%p: when_anyT:start_all(): m_first = %d\n", this, m_first);
            if (m_first)
            {
                for (std::size_t i = 0; i < m_when_any_info_vector.size(); i++)
                {
                    m_when_any_info_vector[i].m_element->start();
                }
                m_first = false;
            }
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(when_anyT& when_any_)
                    : m_when_any(when_any_)
                {
                    clprint(PRI2, "%p: when_anyT::awaiter::awaiter()\n", this);
                }

                bool await_ready()
                {
                    clprint(PRI2, "%p: when_anyT::awaiter::await_ready(); size = %ld\n",
                            this, m_when_any.m_when_any_info_vector.size());
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); i++)
                    {
                        if (m_when_any.m_when_any_info_vector[i].m_when_any_one.get_completed())
                        {
                            clprint(PRI2, "%p: when_anyT::awaiter::await_ready(): return true for i = %d;\n", this, i);
                            return true;
                        }
                    }
                    clprint(PRI2, "%p: when_anyT::awaiter::await_ready(): return false;\n", this);
                    return false;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    clprint(PRI2, "%p: when_anyT::awaiter::await_suspend(...): size = %ld\n",
                            this, m_when_any.m_when_any_info_vector.size());
                    m_when_any.start_all();    // Will have no effect in case of an eager start
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); ++i)
                    {
                        when_any_one * p = &m_when_any.m_when_any_info_vector[i].m_when_any_one;
                        clprint(PRI2, "%p: when_any::awaiter::await_suspend(...): p = %p\n", this, p);
                        p->set_awaiting(awaiting);
                    }
                }

                int await_resume()
                {
                    // Find out which one has completed
                    clprint(PRI2, "%p: when_anyT::awaiter::await_resume(): size = %d\n",
                            this, m_when_any.m_when_any_info_vector.size());
                    int ret = -1;
                    for (std::size_t i = 0; i < m_when_any.m_when_any_info_vector.size(); i++)
                    {
                        if (m_when_any.m_when_any_info_vector[i].m_when_any_one.get_and_mark_as_completed())
                        {
                            clprint(PRI2, "%p: when_anyT::awaiter::await_resume(): return i = %d\n", this, i);
                            // TODO: possibly remove i-th element from m_when_any_one and m_elements
                            // or otherwise mark them as not having to be reconsidered again in await_ready
                            ret = i;
                            break;
                        }
                    }
                    if (ret == -1)
                        clprint(PRI1, "%p: when_anyT::awaiter::await_resume(): Error, none has completed, yet the coroutine has been resumed\n", this);
                    return ret;
                }
            private:
                when_anyT& m_when_any;
            };

            return awaiter{ *this };
        }

    private:
        std::vector<when_any_infoT<TYPE>> m_when_any_info_vector;
        bool m_first{true};
    };

}

#endif
