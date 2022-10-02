/**
 *  Filename: p0100.cpp
 *  Description:
 *  Defines two coroutine types for coroutines that use co_return.
 *  Type 1: syncr<T>: eager coroutine type: the coroutine starts executing upon entry.
 *  Type 2: lazy<T>: lazy coroutine type: upon entry, it immediately returns control
 *                   to its calling function/coroutine that is responsible for resuming
 *                   the lazy coroutine.
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/A%20more%20realistic%20coroutine
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

/**
 * Eager coroutine type.
 *
 */

template<typename T>
struct syncr {
    struct promise_type;
    friend struct promise_type;

    using handle_type = std::coroutine_handle<promise_type>;

    syncr(const syncr&) = delete;

    syncr(syncr&& s)
        : coro(s.coro) {
        print("syncr::syncr(syncr&& s)\n");
        s.coro = nullptr;
    }

    ~syncr() {
        print("~syncr::syncr()\n");
        if (coro) coro.destroy();
    }

    syncr& operator = (const syncr&) = delete;

    syncr& operator = (syncr&& s) {
        print("syncr& syncr::operator = (syncr&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }
    
    T get() {
        print("T syncr::get()\n");
        return coro.promise().value;
    }

    struct promise_type {
        friend struct syncr;

        promise_type() {
            print("syncr::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print("~syncr::promise_type::promise_type()\n");
        }

        auto get_return_object() {
            print("auto syncr::promise_type::get_return_object()\n");
            return syncr<T>{handle_type::from_promise(*this)};
        }
        
        void return_value(T v) {
            print("auto syncr::promise_type::return_value(T v)\n", v );
            value = v;
        }

        auto initial_suspend() {
            print("auto syncr::promise_type::initial_suspend()\n");
            return std::suspend_never{};
        }
            
        auto final_suspend() noexcept {
            print("auto syncr::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            std::exit(1);
        }

    private:
        T value;
    };

public:
    syncr(handle_type h)
        : coro(h) {
        print("syncr::syncr(handle_type h)\n");
    }
    
    handle_type coro;
};

// -----------------------------------------------------------------

template<typename T>
struct lazy {
    struct promise_type;
    friend struct promise_type;

    using handle_type = std::coroutine_handle<promise_type>;

    lazy(const lazy&) = delete;

    lazy(lazy&& s)
        : coro(s.coro) {
        print("lazy::lazy(lazy&& s)\n");
        s.coro = nullptr;
    }

    ~lazy() {
        print("~lazy::lazy()\n");
        if (coro) coro.destroy();
    }

    lazy& operator = (const lazy&) = delete;

    lazy& operator = (lazy&& s) {
        print("lazy& lazy::operator = (lazy&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("T lazy::get() 1\n");
        this->coro.resume();
        print("T lazy::get() 2\n");
        return coro.promise().value;
    }
    
    struct promise_type {
        friend struct lazy;

        promise_type() {
            print("lazy::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print("~lazy::promise_type::promise_type()\n");
        }

        auto get_return_object() {
            print("auto lazy::promise_type::get_return_object()\n");
            return lazy<T>{handle_type::from_promise(*this)};
        }
        
        void return_value(T v) {
            print("void lazy::promise_type::return_value(T v)\n", v );
            value = v;
        }

        auto initial_suspend() {
            print("auto lazy::promise_type::initial_suspend()\n");
            return std::suspend_always{};
        }
        
        auto final_suspend() noexcept {
            print("auto lazy::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            std::exit(1);
        }

    private:
        T value;
    };

public:
    lazy(handle_type h)
        : coro(h) {
        print("lazy::lazy(handle_type h)\n");
    }
    
    handle_type coro;
};

// -----------------------------------------------------------------
/*
The C++ compiler compiles

syncr<int> answer1() {
    co_return 42;
}

into

syncr<int> answer1() {
    syncr<int>::promise_type p;
    auto task = p.initial_suspend();
    syncr<int> ret = p.get_return_object();
    co_await task;

    //co_return 42;
    p.return_value(42);
final_suspend:
    co_await p.final_suspend();
}

The numbers 1 till 4 show the control flow between
the calling function test_syncr() and the called coroutine answer1().
The numbers are followed with trace output.

syncr<T> behaves more or less as a normal function, except
that it "dies" later than a normal function, namely
when it terminates after the final suspend point.
 */

syncr<int> answer1() {
    // 2
    // 00: syncr::promise_type::promise_type()
    // 00: auto syncr::promise_type::initial_suspend()
    // 00: auto syncr::promise_type::get_return_object()
    // 00: syncr::syncr(handle_type h)

    print("syncr<int> answer1()\n");
    
    co_return 42;
    
    // 00: auto syncr::promise_type::return_value(T v)
    // 00: auto syncr::promise_type::final_suspend()
    
    // 4
    // 00: ~syncr::promise_type::promise_type()
}

void test_syncr() {
    // 1
    print("syncr<int> a1 = answer1();\n");
    syncr<int> a1 = answer1();
    
    // 3
    print("int v = a1.get();\n");
    int v = a1.get();
    print("The coroutine value is: %d\n", v);
    
    // 00: ~syncr::syncr()
}

// -----------------------------------------------------------------

lazy<int> answer2() {
    // 2
    
    // 00: lazy::promise_type::promise_type()
    // 00: auto lazy::promise_type::initial_suspend()
    // 00: auto lazy::promise_type::get_return_object()
    // 00: lazy::lazy(handle_type h)

     // 4
    print("lazy<int> answer2()\n");
    
    co_return 42;
    
    // 00: auto lazy::promise_type::return_value(T v)
    // 00: auto lazy::promise_type::final_suspend()
    
    // 6
    // 00: ~lazy::promise_type::promise_type()
}

void test_lazy() {
    // 1
    print("lazy<int> a2 = answer2();\n");
    lazy<int> a2 = answer2();
    
    // 3
    print("int v = a2.get();\n");
    int v = a2.get();
    
    // 00: T lazy::get() 1
    // 00: T lazy::get() 2

    // 5
    print("The coroutine value is: %d\n", v);
    
    // 00: ~lazy::lazy()
}

// -----------------------------------------------------------------

int main() {
    print("test_syncr();\n");
    test_syncr();
    fprintf(stderr, "\n");
    print("test_lazy();\n");
    test_lazy();
    return 0;
}

