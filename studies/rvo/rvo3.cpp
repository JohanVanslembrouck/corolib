/**
 * @file rvo3.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <stdio.h>

#include "rvo.h"
#include "eventhandler.h"

EventHandler eventHandler;

 /**
  * @brief If async_op is passed a number <= 10, it calculates the square of the number itself.
  * Otherwise, it will contact a remote application (bypassed in this implementation) that will do the calculation.
  * async_op is also passed a lambda function that will be called from the event loop when the remote application
  * has sent the response.
  *
  */
void async_op(int i, std::function<void(int)>&& op) {
    if (i <= 10)
        op(i * i);
    else
        eventHandler.set(std::move(op), i * i);
}

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
        printf("%p: async_operation::async_operation(): m_value = %d\n", this, m_value);
    }

    virtual ~async_operation() {
        m_value = -2;
        printf("%p: async_operation::~async_operation(): m_value = %d\n", this, m_value);
    }
	
	async_operation(const async_operation&) = delete;
    async_operation(async_operation&& other) noexcept
        : m_value(other.m_value) {
        printf("%p: async_operation::async_operation(async_operation&&): m_value = %d\n", this, m_value);
        other.m_value = -1;
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

#if 0

/**
 * @brief start_operation is defined as a class with an async_operation object as data member.
 *
 *
 */
class start_operation
{
public:
    start_operation(int i)
    {
        printf("start_operation::start_operation(%d)\n", i);
        start_operation_impl(i);
    }

    int get_value() {
        printf("start_operation::get_value()\n");
        return ao.get_value();
    }

protected:
    void start_operation_impl(int i) {
        printf("start_operation::start_operation_impl(%d)\n", i);
        async_op(i, [this](int v) {
            ao.set_value(v);
        });
    }
private:
    async_operation ao;
};

#else

/**
 * @brief start_operation is now defined as a class inheriting from async_operation.
 * In a full implementation, start_operation will be used as argument to when_all() and when_any().
 * This requires the use of a base class.
 */
class start_operation : public async_operation
{
public:
    start_operation(int i)
    {
        printf("%p: start_operation::start_operation(%d)\n", this, i);
        start_operation_impl(i);
    }

    int get_value() {
        printf("%p: start_operation::get_value()\n", this);
        return async_operation::get_value();
    }

protected:
    void start_operation_impl(int i) {
        printf("%p: start_operation::start_operation_impl(%d)\n", this, i);
        async_op(i, [this](int v) {
            set_value(v);
            });
    }
};

#endif

void test00() {
    // Not applicable
}

void test01() {
    printf("\n--- test01: ---\n");
    start_operation ao(20);
    eventHandler.run();
    int res = ao.get_value();
    printf("test01: res = %d\n", res);
    if (res != 400)
        printf("test01: Expected 400, received res = %d !!!\n", ao.get_value());
}

void test02() {
    printf("\n--- test02: immediate completion ---\n");
    start_operation ao(10);
    eventHandler.run();
    int res = ao.get_value();
    printf("test02: res = %d\n", res);
    if (res != 100)
        printf("test04: Expected 100, received res = %d !!!\n", ao.get_value());
}

void test03a() {
    start_operation ao(20);
    printf("test03a: &ao = %p\n", &ao);
}

void test03() {
    printf("\n--- test03: object goes out of scope ---\n");
    test03a();
    eventHandler.run();
}

start_operation test04a() {
    start_operation ao(20);
    return ao;
}

void test04() {
    printf("\n--- test04: auxiliary function returns operation ---\n");
    start_operation ao = test04a();
    eventHandler.run();
    int res = ao.get_value();
    printf("test04: res = %d\n", res);
    if (res != 400)
        printf("test04: Expected 400, received res = %d !!!\n", ao.get_value());
}

int main() {
    test00();
    test01();
    test02();
    test03();
    test04();
    return 0;
}
