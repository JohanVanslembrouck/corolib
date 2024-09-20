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

#define REPORT_OUT_OF_SCOPE 0

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
        m_index = 0xdeadbeef;

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
#if !USE_IN_MT_APPS
        , m_ready(other.m_ready)
#endif
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
#if !USE_IN_MT_APPS
        m_ready = other.m_ready;
#endif
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
     * @brief async_operation_base::inform_interested_parties is an auxiliary function
     * that informs the coroutine that references this async_operation object using a when_all or a when_any
     * that the operation has completed.
     * If there are no interested parties (yet), m_ready is set to true.
     * The function is called from async_operation_base::completed.
     * See async_task_base<TYPE>::promise_type::inform_interested_parties 
     * and async_task_void::promise_type::inform_interested_parties for similar functions.
     */
    void async_operation_base::inform_interested_parties()
    {
        if (m_ctr)
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

    /**
     * @brief async_operation_base::completed
     */
    void async_operation_base::completed()
    {
        print(PRI2, "%p: async_operation_base::completed(): m_index = %d = 0x%x\n", this, m_index, m_index);

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
#if REPORT_OUT_OF_SCOPE
                    // See explanation below
                    if (m_index == std::make_signed_t<int>(0xdeadbeef) || m_index == std::make_signed_t<int>(0xdddddddd))
                        print(PRI1, "%p: async_operation_base::completed(): after tmp[%d].resume(); ACCESSING OUT-OF-SCOPE MEMORY!!!\n", this);
                    else
                        print(PRI2, "%p: async_operation_base::completed(): m_index = %d, after tmp[%d].resume();\n", this, m_index, i);
#endif
                }
                m_ready = true;

                hasCompleted = true;
            }
        }
        else
        {
            if (m_awaiting)
            {
                print(PRI2, "%p: async_operation_base::completed(): m_index = %d, before m_awaiting.resume();\n", this, m_index);
                m_awaiting.resume();
#if REPORT_OUT_OF_SCOPE
                // async_operations are usually (always) created as local variables in a coroutine,
                // i.e. these objects are placed on the stack.
                // When an async_operation completes, the coroutine to which it belongs may also be destroyed.
                // The coroutine's stack frame will be released, including the space for the asyn_operation object.
                // The async_operation's destructor writes 0xdeadbeef to m_index.
                // On Windows, the OS will fill the content of the released stack frame with 0xdddddddd (for 4 bytes).
                // The following print statements can be used to identify those places where this occurs.
                // There should be no reason to access async_operation data members after the resume() call.
                if (m_index == std::make_signed_t<int>(0xdeadbeef) || m_index == std::make_signed_t<int>(0xdddddddd))
                    print(PRI1, "%p: async_operation_base::completed(): after m_awaiting.resume(); ACCESSING OUT-OF-SCOPE MEMORY!!!\n", this);
                else
                    print(PRI2, "%p: async_operation_base::completed(): m_index = %d, after m_awaiting.resume();\n", this, m_index);
#endif
                hasCompleted = true;
            }
        }
        
        if (!hasCompleted)
        {
            inform_interested_parties();
        }
    }
   
    void resume_multiple_coroutines(bool allow)
    {
        _resume_multiple_coroutines = allow;
    }
    
    bool _resume_multiple_coroutines = false;
}
