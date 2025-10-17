/**
 * @file commservice.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "corolib/print.h"
#include "corolib/commservice.h"

int next_pow2(int x) {
    int p = 1;
    while (p < x)
        p <<= 1;
    return p;
}

namespace corolib
{

async_operation_base CommService::reserved;

CommService::CommService(int nr_operations)
    : m_nr_operations(next_pow2(nr_operations))
    , m_index(-1)
{
    clprint(PRI2, "%p: CommService::CommService()\n", this);

    m_async_operation_info = std::make_unique<async_operation_info[]>(m_nr_operations);
    for (int i = 0; i < m_nr_operations; i++)
    {
        m_async_operation_info[i].async_operation = nullptr;
        m_async_operation_info[i].start = std::chrono::high_resolution_clock::now();
    }
}

CommService::~CommService()
{
    clprint(PRI2, "%p: CommService::~CommService()\n", this);
    m_nr_operations = 0;
    m_index = -1;
}

int CommService::get_free_index_ts()
{
	return get_free_index();
}

int CommService::get_free_index()
{
    clprint(PRI3, "%p: CommService::get_free_index_ts(): m_index = %d\n", this, m_index);
    for (int i = 0; i < NROPERATIONS / 2; i++)
    {
        m_index = (m_index + 1) & (NROPERATIONS - 1);
        clprint(PRI3, "%p: CommService::get_free_index_ts(): m_index = %d\n", this, m_index);
        if (m_async_operation_info[m_index].async_operation == nullptr)
        {
            // Found free entry
            clprint(PRI2, "%p: CommService::get_free_index_ts() returns %d\n", this, m_index);
			m_async_operation_info[m_index].async_operation = &reserved;
            return m_index;
        }
    }
    // No more free entries
    clprint(PRI1, "%p: CommService::get_free_index_ts() returns -1!\n", this);
    return -1;
}

void CommService::add_entry(int index, async_operation_base* op, bool timestamp)
{
    if (index == -1)
    {
        clprint(PRI1, "%p: CommService::add_entry(): index == -1!\n", this);
        return;
    }

    if (m_async_operation_info[index].async_operation == &reserved)
    {
        // Entry is still free
        m_async_operation_info[index].async_operation = op;
        if (timestamp)
            m_async_operation_info[index].start = std::chrono::high_resolution_clock::now();
    }
    else
    {
        clprint(PRI1, "%p: CommService::add_entry(): m_async_operation_info[%d].async_operation WRONGLY INITIALIZED!!!\n", this, index);
    }
}

void CommService::update_entry(int index, async_operation_base* op, bool timestamp)
{
    clprint(PRI2, "%p: CommService::update_entry(index = %d, op = %p, timestamp = %d)\n",
                                        this, index, op, static_cast<int>(timestamp));
    if (index == -1)
    {
        clprint(PRI1, "%p: CommService::update_entry(index = -1, op = %d, timestamp = %d): index == -1!\n",
                                        this, index, op, static_cast<int>(timestamp));
        return;
    }

	m_async_operation_info[index].async_operation = op;
}

}
