/**
 * @file rvo2.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "rvo.h"
#include "eventhandler.h"

class async_operation;

/**
 * @brief async_operations
 *
 *
 */
class async_operations
{
public:
    static const int NROPERATIONS = 4;

    int get_free_index() {
        for (int i = 0; i < NROPERATIONS; i++) {
            m_index = (m_index + 1) & (NROPERATIONS - 1);
            if (m_async_operations[m_index] == nullptr) {
                return m_index;
            }
        }
        return -1;
    }

    void add_entry(int index, async_operation* ao) {
        if (index >= 0 && index < NROPERATIONS)
            m_async_operations[index] = ao;
    }

    void release_entry(int index) {
        if (index >= 0 && index < NROPERATIONS)
            m_async_operations[index] = nullptr;
    }

    async_operation* get_entry(int index) {
        return m_async_operations[index];
    }

private:
    async_operation* m_async_operations[NROPERATIONS] = { nullptr, nullptr, nullptr, nullptr };
    int m_index = -1;
};

async_operations async_ops;

/**
 * @brief async_operation
 *
 *
 */
class async_operation
{
public:
    async_operation(int idx)
        : m_index(idx) {
		printf("%p: async_operation::async_operation(%d)\n", this, idx);
        async_ops.add_entry(m_index, this);
	}

	virtual ~async_operation() {
        printf("%p: async_operation::~async_operation(%d)\n", this, m_index);
        if (m_index >- 0)
            async_ops.release_entry(m_index);
        m_value = -1;
        m_index = -1;
	}
	
	async_operation(const async_operation&) = delete;
    async_operation(async_operation&& other) noexcept
            : m_value(other.m_value)
            , m_index(other.m_index)
        {
        printf("%p: async_operation::async_operation(async_operation&&)\n", this );
        async_ops.add_entry(m_index, this);
        other.m_value = -1;
        other.m_index = -1;
	}
	
    async_operation& operator = (const async_operation&) = delete;
    async_operation& operator = (async_operation&& other) noexcept = delete;

    void set_value(int value) {
        printf("%p: async_operation::set_value(%d)\n", this, value);
        m_value = value;
    }

    int get_value() {
        printf("%p: async_operation::get_value()\n", this);
        return m_value;
    }

    int m_value = -1;
    int m_index = -1;
};

/**
 * @brief start_operation and start_operation_impl
 * 
 * 
 */
void start_operation_impl(int i, int index) {
    async_op(i, [index](int i) {
        async_operation* ao = async_ops.get_entry(index);
        printf("start_operation_impl: index = %d, ao = %p\n", index, ao);
        if (ao)
            ao->set_value(i);
        });
}

async_operation start_operation(int i) {
    int idx = async_ops.get_free_index();
    async_operation ret(idx);
    printf("%p = &ret: start_operation, idx = %d\n", &ret, idx);
    start_operation_impl(i, idx);
    return ret;
}

void test00() {
}

#include "rvo.cpp"
