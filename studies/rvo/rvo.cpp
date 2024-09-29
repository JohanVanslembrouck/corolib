/**
 * @file rvo.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

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

void test01() {
    printf("\n--- test01: test RVO ---\n");
    async_operation ao = start_operation(20);
    printf("test01: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test01: res = %d\n", ao.get_value());
}

void test02() {
    printf("\n--- test02: test RVO: immediate completion ---\n");
    async_operation ao = start_operation(10);
    printf("test02: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test02: res = %d\n", ao.get_value());
}

void test03a() {
    async_operation ao = start_operation(20);
    printf("test03a: &ao = %p\n", &ao);
}

void test03() {
    printf("\n--- test03: object goes out of scope ---\n");
    test03a();
    eventHandler.run();
}

async_operation test04a() {
    async_operation ao = start_operation(20);
    printf("test04a: &ao = %p\n", &ao);
    return ao;
}

void test04() {
    printf("\n--- test04: auxiliary function returns async_operation ---\n");
    async_operation ao = test04a();
    eventHandler.run();
    printf("test04: res = %d\n", ao.get_value());
}

int main() {
    test00();
    test01();
    test02();
    test03();
    test04();
	return 0;
}
