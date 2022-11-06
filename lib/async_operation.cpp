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
        , m_awaiting(nullptr)           // initialized in await_suspend()
        , m_ctr(nullptr)
        , m_waitany(nullptr)
        , m_index(index)
        , m_ready(false)                // set to true in completed()
        , m_autoreset(false)
        , m_timestamp(timestamp)
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(CommService* s = %p, index = %d)\n", this, s, index);
		
        if (m_service)
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

        if (m_service)
        {
            m_service->update_entry(m_index, nullptr, m_timestamp);
        }
        m_index = -1;

        m_awaiting = nullptr;
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
    async_operation_base::async_operation_base(async_operation_base&& s) noexcept
        : m_service(s.m_service)
        , m_awaiting(s.m_awaiting)
        , m_ctr(s.m_ctr)
        , m_waitany(s.m_waitany)
        , m_index(s.m_index)
        , m_ready(s.m_ready)
        , m_autoreset(s.m_autoreset)
        , m_timestamp(s.m_timestamp)      // Should be the same: we cannot "move" from implementation.
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(async_operation_base&& s): s.m_index = %d\n", this, s.m_index);

        // Tell the CommService we are at another address after the move.
        if (m_service)
        {
            m_service->update_entry(m_index, this, m_timestamp);
        }

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_ctr = nullptr;
        s.m_waitany = nullptr;
        s.m_index = -1;        // indicates move
        s.m_ready = false;
        s.m_autoreset = false;
        s.m_timestamp = false;
    }

    /**
     * @brief async_operation_base::operator =
     * @param s
     * @return async_operation_base
     */
    async_operation_base& async_operation_base::operator = (async_operation_base&& s) noexcept
    {
        print(PRI2, "%p: async_operation_base::operator = (async_operation_base&& s): m_index = %d, s.m_index = %d\n", this, m_index, s.m_index);

        // Clean our entry at the original location, because we will move to another one.
        if (m_service)
        {
            m_service->update_entry(m_index, nullptr, m_timestamp);
        }

        m_service = s.m_service;
        m_awaiting = s.m_awaiting;
 
        // The following 2 tests allow an async_operation that takes part in
        // a when_all or when_any to be re-assigned.
        // This avoids disposing the original when_all or when_any
        // and constructing a new one.

        if (m_ctr != nullptr && s.m_ctr == nullptr)
            ; // do not overwrite m_ctr
        else
            m_ctr = s.m_ctr;

        if (m_waitany != nullptr && s.m_waitany == nullptr)
            ; // do not overwrite m_waitany
        else
            m_waitany = s.m_waitany;

        m_index = s.m_index;
        m_ready = s.m_ready;
        m_autoreset = s.m_autoreset;
        //m_timestamp = s.m_timestamp;	// Should be the same: we cannot "move" from implementation.

        // Tell the CommService we are at another address after the move.
        if (m_service)
        {
            m_service->update_entry(m_index, this, m_timestamp);
        }

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_ctr = nullptr;
        s.m_waitany = nullptr;
        s.m_index = -1;        // indicates move
        s.m_ready = false;
        s.m_autoreset = false;
        s.m_timestamp = false;

        return *this;
    }

    /**
     * @brief async_operation_base::completed
     */
    void async_operation_base::completed()
    {
        print(PRI2, "%p: async_operation_base::completed()\n", this);
        if (m_awaiting)
        {
            print(PRI2, "%p: async_operation_base::completed(): before m_awaiting.resume();\n", this);
            m_awaiting.resume();
            m_ready = true;
            print(PRI2, "%p: async_operation_base::completed(): after m_awaiting.resume();\n", this);
        }
        else if (m_ctr)
        {
            print(PRI2, "%p: async_operation_base::completed(): before m_ctr->completed();\n", this);
            m_ctr->completed();
            print(PRI2, "%p: async_operation_base::completed(): after m_ctr->completed();\n", this);
        }
        else if (m_waitany)
        {
            print(PRI2, "%p: async_operation_base::completed(): before m_waitany->completed();\n", this);
            m_waitany->completed();
            print(PRI2, "%p: async_operation_base::completed(): after m_waitany->completed();\n", this);
        }
        else
        {
            print(PRI2, "%p: async_operation_base::completed(): m_awaiting not yet initialized!!!\n", this);
            print(PRI2, "%p: async_operation_base::completed(): operation completed before co_waited!!\n", this);
            m_ready = true;     // Set to completed.
        }
    }
}
