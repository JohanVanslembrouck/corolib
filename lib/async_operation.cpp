/**
 * @file async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <thread>

#include "corolib/async_operation.h"
#include "corolib/print.h"
#include "corolib/commservice.h"

namespace corolib
{
    /**
     * @brief async_operation_base::async_operation_base
     * @param s
     * @param index
     */
    async_operation_base::async_operation_base(CommService* s, int index, bool timestamp)
        : m_service(s)
        , m_awaitings{}                 // RESUME_MULTIPLE_COROUTINES
        , m_awaiting(nullptr)           // !RESUME_MULTIPLE_COROUTINES
        , m_ctr(nullptr)
        , m_waitany(nullptr)
        , m_index(index)
        , m_ready(false)                // set to true in completed()
        , m_autoreset(false)
        , m_timestamp(timestamp)
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(CommService* s = %p, index = %d, timestamp = %d)\n",
            this, s, index, static_cast<int>(timestamp));
        
        if (m_service && m_index != -1)
        {
            m_service->add_entry(m_index, this, m_timestamp);
        }
    }

    /**
     * @brief async_operation_base::~async_operation_base
     */
    async_operation_base::~async_operation_base()
    {
        print(PRI2, "%p: async_operation_base::~async_operation_base(): m_index = %d\n", this, m_index);

        if (m_service && m_index != -1)
        {
            m_service->update_entry(m_index, nullptr, m_timestamp);
        }
        m_index = -1;

        // m_awaitings          // RESUME_MULTIPLE_COROUTINES
        m_awaiting = nullptr;   // !RESUME_MULTIPLE_COROUTINES
        m_ctr = nullptr;
        m_waitany = nullptr;
        m_service = nullptr;
        m_ready = false;
        m_autoreset = false;
        m_timestamp = false;
    }

    /**
     * @brief async_operation_base::async_operation_base
     * @param s
     */
    async_operation_base::async_operation_base(async_operation_base&& other) noexcept
        : m_service(other.m_service)
        , m_awaitings(std::move(other.m_awaitings))         // RESUME_MULTIPLE_COROUTINES
        , m_awaiting(other.m_awaiting)                      // !RESUME_MULTIPLE_COROUTINES
        , m_ctr(other.m_ctr)
        , m_waitany(other.m_waitany)
        , m_index(other.m_index)
        //, m_ready(other.m_ready)
        , m_autoreset(other.m_autoreset)
        , m_timestamp(other.m_timestamp)      // Should be the same: we cannot "move" from implementation.
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(async_operation_base&& s): other.m_index = %d, other.m_service = %p, other.m_timestamp = %d\n",
                this, other.m_index, other.m_service, static_cast<int>(other.m_timestamp));

        // Tell the CommService we are at another address after the move.
        if (m_service && m_index != -1)
        {
            m_service->update_entry(m_index, this, m_timestamp);
        }

        other.cleanup();
    }

    /**
     * @brief async_operation_base::operator =
     * @param s
     * @return async_operation_base
     */
    async_operation_base& async_operation_base::operator = (async_operation_base&& other) noexcept
    {
        print(PRI2, "%p: async_operation_base::operator = (async_operation_base&& other):\n\tm_index = %d, other.m_index = %d, other.m_service = %p, other.m_timestamp = %d\n",
            this, m_index, other.m_index, other.m_service, static_cast<int>(other.m_timestamp));

        if (m_timestamp != other.m_timestamp)
            if (m_timestamp == 1)
                // We should not go from timestamped to non-timestamped.
                // Going from non-timestamped to timestamped can occur when the async_operation is declared without
                // immediately initializing it to an operation (e.g. when it is declared in an array).
                print(PRI1, "%p: async_operation_base::operator = (async_operation_base&& other): m_timestamp = %d != other.m_timestamp = %d\n",
                    this, static_cast<int>(m_timestamp), static_cast<int>(other.m_timestamp));

        m_timestamp = other.m_timestamp;

        // Clean our entry at the original location, because we will move to another one.
        if (m_service && m_index != -1)
        {
            m_service->update_entry(m_index, nullptr, m_timestamp);
        }

        m_service = other.m_service;
        m_awaitings = std::move(other.m_awaitings);     // RESUME_MULTIPLE_COROUTINES
        m_awaiting = other.m_awaiting;                  // !RESUME_MULTIPLE_COROUTINES
 
        // The following 2 tests allow an async_operation that takes part in
        // a when_all or when_any to be re-assigned.
        // This avoids disposing the original when_all or when_any
        // and constructing a new one.

        if (m_ctr != nullptr && other.m_ctr == nullptr)
            ; // do not overwrite m_ctr
        else
            m_ctr = other.m_ctr;

        if (m_waitany != nullptr && other.m_waitany == nullptr)
            ; // do not overwrite m_waitany
        else
            m_waitany = other.m_waitany;

        m_index = other.m_index;
        //m_ready = other.m_ready;
        m_autoreset = other.m_autoreset;

        // Tell the CommService we are at another address after the move.
        if (m_service)
        {
            m_service->update_entry(m_index, this, m_timestamp);
        }

        other.cleanup();
        return *this;
    }

    /**
    * @brief async_operation_base::cleanup cleans the original object after a move
    */
    void async_operation_base::cleanup()
    {
        m_service = nullptr;
        m_awaitings.clear();            // RESUME_MULTIPLE_COROUTINES
        m_awaiting = nullptr;           // !RESUME_MULTIPLE_COROUTINES
        m_ctr = nullptr;
        m_waitany = nullptr;
        m_index = -1;        // indicates move
        m_ready = false;
        m_autoreset = false;
        m_timestamp = false;
    }

    /**
     * @brief async_operation_base::completed
     */
    void async_operation_base::completed()
    {
        print(PRI2, "%p: async_operation_base::completed(): m_index = %d\n", this, m_index);

        bool hasCompleted = false;
        if (corolib::_resume_multiple_coroutines)
        {
            if (m_awaitings.size())
            {
                std::vector<std::coroutine_handle<>> tmp;

                print(PRI2, "%p: async_operation_base::completed(): m_index = %d, m_awaitings.size() = %d\n", this, m_index, m_awaitings.size());

                for (std::size_t i = 0; i < m_awaitings.size(); i++)
                {
                    tmp.push_back(m_awaitings[i]);
                }

                m_awaitings.clear();

                print(PRI2, "%p: async_operation_base::completed(): m_index = %d, tmp.size() = %d\n", this, m_index, tmp.size());

                for (std::size_t i = 0; i < tmp.size(); i++)
                {
                    print(PRI2, "%p: async_operation_base::completed(): before tmp[%d].resume();\n", this, i);
                    tmp[i].resume();
                    print(PRI2, "%p: async_operation_base::completed(): after tmp[%d].resume();\n", this, i);
                }
                m_ready = true;

                hasCompleted = true;
            }
        }
        else
        {
            if (m_awaiting)
            {
                print(PRI2, "%p: async_operation_base::completed(): : m_index = %d, before m_awaiting.resume();\n", this, m_index);
                m_awaiting.resume();
                print(PRI2, "%p: async_operation_base::completed(): m_index = %d, after m_awaiting.resume();\n", this, m_index);

                hasCompleted = true;
            }
        }
        
        if (hasCompleted)
            ; // already handled above
        else if (m_ctr)
        {
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, before m_ctr->completed();\n", this, m_index);
            std::coroutine_handle<> handle = m_ctr->completed();
            handle.resume();
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, after m_ctr->completed();\n", this, m_index);
        }
        else if (m_waitany)
        {
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, before m_waitany->completed();\n", this, m_index);
            std::coroutine_handle<> handle = m_waitany->completed();
            handle.resume();
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, after m_waitany->completed();\n", this, m_index);
        }
        else
        {
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, m_awaiting not (yet) initialized!\n", this, m_index);
            print(PRI2, "%p: async_operation_base::completed(): m_index = %d, operation completed before co_waited!\n", this, m_index);
            m_ready = true;     // Set to completed.
        }
    }
   
    void resume_multiple_coroutines(bool allow)
    {
        _resume_multiple_coroutines = allow;
    }
    
    bool _resume_multiple_coroutines = false;
}
