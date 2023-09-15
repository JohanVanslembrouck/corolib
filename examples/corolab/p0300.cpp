/**
 *  Filename: p0300.cpp
 *  Description: Illustrates use of co_yield to define a generator type.
 *  The generator is independent of the applications in which it is used.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/Yielding%20Generators
 */

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>

#include "print0.h"

// -----------------------------------------------------------------

#include <coroutine>

template <typename T>
struct generator {

    struct promise_type {

        promise_type() : current_value(0) {
            print("generator::promise_type::promise_type()\n");
        }
        ~promise_type() {
            print("generator::promise_type::~promise_type()\n");
        }

        auto initial_suspend() {
            print("generator::promise_type::initial_suspend()\n");
            return std::suspend_always{};
        }

        auto final_suspend() noexcept {
            print("generator::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        auto get_return_object() {
            print("auto generator::promise_type::get_return_object()\n");
            return generator{ handle_type::from_promise(*this) };
        }

        void unhandled_exception() {
            print("auto generator::promise_type::unhandled_exception()\n");;
            std::exit(1);
        }

        auto return_void() {
            print("auto generator::promise_type::return_void\n");
            return std::suspend_never{};
        }

        auto yield_value(int value) {
            print("auto generator::promise_type::yield_value(%d)\n", value);
            current_value = value;
            return std::suspend_always{};
        }

        T current_value;
    };
    using handle_type =
        std::coroutine_handle<promise_type>;

public:
    ~generator() {
        print("generator::~generator(): %s\n", (!coro ? "(empty)" : "(contains a coroutine)"));
        if (coro) {
            print("generator::~generator(): coro.done() = %d\n", coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    generator(const generator&) = delete;

    generator(generator&& g)
        : coro(g.coro) {
        print("generator::generator(generator&& g)\n");
        g.coro = nullptr;
    };

    bool move_next() {
        print("bool generator::move_next()\n");
        coro.resume();
        auto still_going = !coro.done();
        print("  Are we done? %s\n", (still_going ? "There is another" : "We're done"));
        return still_going;
    }

    T current_value() {
        print("T generator::current_value()\n");
        return coro.promise().current_value;
    }

private:
    generator(handle_type h)
        : coro(h) {
        print("generator::generator(handle_type h)\n");
    }

    handle_type coro;
};

// -----------------------------------------------------------------

generator<int> count() {
    print("count(): Going to yield 1\n");
    co_yield 1;
    print("count(): Going to yield 2\n");
    co_yield 2;
    print("count(): Going to yield 3\n");
    co_yield 3;
    print("count(): count() is done\n");
}

void test_count() {
    auto g = count();
    do {
        print(">> %d\n", g.current_value());
    } while (g.move_next());
}

// -----------------------------------------------------------------

// a non-optimized way of checking for prime numbers:
bool is_prime(int x) {
    for (int i = 2; i < x; ++i)
        if (x % i == 0)
            return false;
    return true;
}

generator<int> primes(int limit) {
    if (limit >= 2)
        co_yield 2;
    for (int n = 3; n <= limit; n += 2) {
        if (is_prime(n))
            co_yield n;
    }
}

void test_primes() {
    auto g = primes(100);
    do {
        print(">> %d\n", g.current_value());
    } while (g.move_next());
}

// -----------------------------------------------------------------

int main() {
    print("test_count()\n");
    test_count();
    fprintf(stderr, "\n");
    print("test_primes()\n");
    test_primes();
    return 0;
}

