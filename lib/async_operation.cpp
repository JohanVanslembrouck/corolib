/**
 * @file async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <thread>

#include "corolib/async_operation.h"
#include "corolib/print.h"

namespace corolib
{
    /**
     * @brief async_operation_base::async_operation_base
     * @param s
     * @param index
     */
    async_operation_base::async_operation_base(CommService* s, int index)
        : m_service(s)
        , m_awaiting(nullptr)            // initialized in await_suspend()
        , m_ready(false)                // set to true in completed()
        , m_autoreset(false)
        , m_index(index)
        , m_ctr(nullptr)
        , m_waitany(nullptr)
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(CommService* s = %p, index = %d)\n", this, s, index);
        if (m_service)
        {
            if (m_service->m_async_operations[m_index] == nullptr)
            {
                m_service->m_async_operations[m_index] = this;
            }
            else
            {
                print(PRI1, "%p: async_operation_base::async_operation_base(): m_service->m_async_operations[%d] WRONGLY INITIALIZED!!!\n", this, m_index);
            }
        }
    }

    /**
     * @brief async_operation_base::~async_operation_base
     */
    async_operation_base::~async_operation_base()
    {
        print(PRI2, "%p: async_operation_base::~async_operation_base(): m_index = %d\n", this, m_index);
        m_ready = false;
        m_autoreset = false;
        if (m_index != -1)
        {
            if (m_service)
            {
                m_service->m_async_operations[m_index] = nullptr;
            }
        }
        m_index = -1;
        m_ctr = nullptr;
        m_waitany = nullptr;
    }

    /**
     * @brief async_operation_base::async_operation_base
     * @param s
     */
    async_operation_base::async_operation_base(async_operation_base&& s) noexcept
        : m_service(s.m_service)
        , m_awaiting(s.m_awaiting)
        , m_ready(s.m_ready)
        , m_autoreset(s.m_autoreset)
        , m_index(s.m_index)
        , m_ctr(s.m_ctr)
        , m_waitany(s.m_waitany)
    {
        print(PRI2, "%p: async_operation_base::async_operation_base(async_operation_base&& s): s.m_index = %d\n", this, s.m_index);

        // Tell the CommService we are at another address after the move.
        m_service->m_async_operations[m_index] = this;

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        s.m_autoreset = false;
        s.m_index = -1;        // indicates move
        s.m_ctr = nullptr;
        s.m_waitany = nullptr;
    }

    /**
     * @brief async_operation_base::operator =
     * @param s
     * @return
     */
    async_operation_base& async_operation_base::operator = (async_operation_base&& s) noexcept
    {
        print(PRI2, "%p: async_operation_base::async_operation_base = (async_operation_base&& s): m_index = %d, s.m_index = %d\n", this, m_index, s.m_index);

        // Clean our entry at the original location, because we will move to another one.
        if (m_service)
            m_service->m_async_operations[m_index] = nullptr;

        m_service = s.m_service;
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        m_autoreset = s.m_autoreset;
        m_index = s.m_index;

        // The following 2 tests allow an async_operation that takes part in
        // a wait_all or wait_any to be re-assigned.
        // This avoids disposing the original wait_all or wait_any
        // and constructing a new one.

        if (m_ctr != nullptr && s.m_ctr == nullptr)
            ; // do not overwrite m_ctr
        else
            m_ctr = s.m_ctr;

        if (m_waitany != nullptr && s.m_waitany == nullptr)
            ; // do not overwrite m_waitany
        else
            m_waitany = s.m_waitany;

        // Tell the CommService we are at another address after the move.
        if (m_service)
            m_service->m_async_operations[m_index] = this;

        s.m_service = nullptr;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        s.m_autoreset = false;
        s.m_index = -1;        // indicates move
        s.m_ctr = nullptr;
        s.m_waitany = nullptr;

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
