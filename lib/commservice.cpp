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
    for (int i = 0; i < NROPERATIONS; i++)
    {
        m_async_operations[i] = nullptr;
    }

    for (int i = 0; i < NROPERATIONS; i++)
    {
        m_async_operation_info[i].async_operation = nullptr;
        m_async_operation_info[i].start = std::chrono::high_resolution_clock::now();
    }
}

CommService::~CommService()
{
}

int CommService::get_free_index()
{
    for (int i = 0; i < NROPERATIONS / 2; i++)
    {
        m_index = (m_index + 1) & (NROPERATIONS - 1);
        if (m_async_operations[m_index] == nullptr)
        {
            // Found free entry
            return m_index;
        }
    }
    // No more free entries
    assert(m_async_operations[m_index] == nullptr);
    return -1;
}

int CommService::get_free_index_ts()
{
    for (int i = 0; i < NROPERATIONS / 2; i++)
    {
        m_index_ts = (m_index_ts + 1) & (NROPERATIONS - 1);
        if (m_async_operation_info[m_index_ts].async_operation == nullptr)
        {
            // Found free entry
            return m_index_ts;
        }
    }
    // No more free entries
    assert(m_async_operation_info[m_index_ts].async_operation == nullptr);
    return -1;
}

}
