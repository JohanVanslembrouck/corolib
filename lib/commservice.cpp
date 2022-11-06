/**
 * @file commservice.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <chrono>

#include <corolib/print.h>
#include <corolib/commservice.h>

namespace corolib
{

CommService::CommService()
    : m_index(-1)
    , m_index_ts(-1)
{
    print(PRI2, "%p: CommService::CommService()\n", this);

    // Initializing the old implementation
    for (int i = 0; i < NROPERATIONS; i++)
    {
        m_async_operations[i] = nullptr;
    }

    // Initializing the new implementation
    for (int i = 0; i < NROPERATIONS; i++)
    {
        m_async_operation_info[i].async_operation = nullptr;
        m_async_operation_info[i].start = std::chrono::high_resolution_clock::now();
    }
}

CommService::~CommService()
{
    print(PRI2, "%p: CommService::~CommService()\n", this);
}

int CommService::get_free_index()
{
    for (int i = 0; i < NROPERATIONS / 2; i++)
    {
        m_index = (m_index + 1) & (NROPERATIONS - 1);
        print(PRI3, "%p: CommService::get_free_index(): m_index = %d\n", this, m_index);
        if (m_async_operations[m_index] == nullptr)
        {
            // Found free entry
            print(PRI2, "%p: CommService::get_free_index() returns %d\n", this, m_index);
            return m_index;
        }
    }
    // No more free entries
    print(PRI1, "%p: CommService::get_free_index_ts() returns -1!\n", this, m_index);
    return -1;
}

int CommService::get_free_index_ts()
{
    for (int i = 0; i < NROPERATIONS / 2; i++)
    {
        m_index_ts = (m_index_ts + 1) & (NROPERATIONS - 1);
        print(PRI3, "%p: CommService::get_free_index_ts(): m_index_ts = %d\n", this, m_index_ts);
        if (m_async_operation_info[m_index_ts].async_operation == nullptr)
        {
            // Found free entry
            print(PRI2, "%p: CommService::get_free_index_ts() returns %d\n", this, m_index_ts);
            return m_index_ts;
        }
    }
    // No more free entries
    print(PRI1, "%p: CommService::get_free_index_ts() returns -1!\n", this);
    return -1;
}

void CommService::add_entry(int index, async_operation_base* op, bool timestamp)
{
    if (index == -1)
    {
        print(PRI1, "%p: CommService::add_entry(): index == -1!\n", this);
        return;
    }

    if (!timestamp)
    {
        if (m_async_operations[index] == nullptr)
        {
            // Entry is still free
            m_async_operations[index] = op;
        }
        else
        {
            print(PRI1, "%p: CommService::add_entry(): m_async_operations[%d] WRONGLY INITIALIZED!!!\n", this, index);
        }
    }
    else
    {
        if (m_async_operation_info[index].async_operation == nullptr)
        {
            // Entry is still free
            m_async_operation_info[index].async_operation = op;
            m_async_operation_info[index].start = std::chrono::high_resolution_clock::now();
        }
        else
        {
            print(PRI1, "%p: CommService::add_entry(): m_async_operation_info[%d].async_operation WRONGLY INITIALIZED!!!\n", this, index);
        }
    }
}

void CommService::update_entry(int index, async_operation_base* op, bool timestamp)
{
    if (index == -1)
    {
        print(PRI1, "%p: CommService::update_entry(): index == -1!\n", this);
        return;
    }

    if (!timestamp)
        m_async_operations[index] = op;
    else
        m_async_operation_info[index].async_operation = op;
}

}
