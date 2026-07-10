/**
 * @file rvo1.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <stdio.h>

#include "rvo.h"
#include "eventhandler.h"

 /**
  * @brief async_operation
  *
  *
  */
class async_operation
{
public:
	async_operation() 
        : m_value(-1) {
		printf("%p: async_operation::async_operation(): m_value = %d\n", this, m_value );
	}

	virtual ~async_operation() {
        m_value = -2;
		printf("%p: async_operation::~async_operation(): m_value = %d\n", this, m_value );
	}
	
	async_operation(const async_operation&) = delete;
    async_operation(async_operation&& other) noexcept
        : m_value(other.m_value) {
        printf("%p: async_operation::async_operation(async_operation&&): m_value = %d\n", this, m_value );
        other.m_value = -2;
	}
	
    async_operation& operator = (const async_operation&) = delete;
    async_operation& operator = (async_operation&& other) noexcept {
        m_value = other.m_value;
        printf("%p: async_operation::operator = (async_operation&&): m_value = %d\n", this, m_value);
        other.m_value = -2;
        return *this;
    }

    void set_value(int value) {
        // printf("%p: async_operation::set_value(%d): m_value = %d\n", this, value, m_value);
        if (m_value != -1)
            printf("%p: async_operation::set_value(%d): async_operation has gone out of scope: m_value = %d\n", this, value, m_value);
        else
            printf("%p: async_operation::set_value(%d)\n", this, value);
        m_value = value;
    }

    int get_value() {
        if (m_value == -2)
            printf("%p: async_operation::get_value() async_operation has gone out of scope and returns %d\n", this, m_value);
        else
            printf("%p: async_operation::get_value() returns %d\n", this, m_value);
        return m_value;
    }

private:
    int m_value;
};

/**
 * @brief start_operation and start_operation_impl
 *
 *
 */
void start_operation_impl(int i, async_operation* ao) {
    printf("%p = ao: start_operation_impl\n", ao);
    async_op(i, [ao](int v) {
        if (ao)
            ao->set_value(v);
        });
}

async_operation start_operation(int i) {
    async_operation ret;
    start_operation_impl(i, &ret);
    printf("%p = &ret: start_operation\n", &ret);
    return ret;
}

void test00() {
    printf("\n--- test00: pass object as argument ---\n");
    async_operation ao;
    start_operation_impl(20, &ao);
    eventHandler.run();
    int res = ao.get_value();
    printf("test00: res = %d\n", res);
    if (res != 400)
        printf("test04: Expected 400, received res = %d !!!\n", ao.get_value());
}

#include "rvo.cpp"
