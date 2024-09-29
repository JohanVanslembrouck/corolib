/**
 * @file rvo1.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
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
	async_operation() {
		printf("%p: async_operation::async_operation()\n", this );
	}
	virtual ~async_operation() {
		printf("%p: async_operation::~async_operation()\n", this );
	}
	
	async_operation(const async_operation&) = delete;
    async_operation(async_operation&& other) noexcept
        : m_value(other.m_value) {
        printf("%p: async_operation::async_operation(async_operation&&)\n", this );
        other.m_value = -1;
	}
	
    async_operation& operator = (const async_operation&) = delete;
    async_operation& operator = (async_operation&&) noexcept = delete;

    void set_value(int value) {
        printf("%p: async_operation::set_value(%d)\n", this, value);
        m_value = value;
    }

    int get_value() {
        printf("%p: async_operation::get_value()\n", this);
        return m_value;
    }

    int m_value = -1;
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
    printf("test00: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test00: res = %d\n", ao.get_value());
}

#include "rvo.cpp"
