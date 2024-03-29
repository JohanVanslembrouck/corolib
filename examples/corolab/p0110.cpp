/**
 *  Filename: p0110.cpp
 *  Description:
 *  Defines two coroutine types for coroutines that use co_return.
 *  Type 1: syncr<T>: eager coroutine type: the coroutine starts executing upon entry.
 *  Type 2: lazy<T>: lazy coroutine type: upon entry, it immediately returns control
 *                   to its calling function/coroutine that is responsible for resuming
 *                   the lazy coroutine.
 *  Same functionality as p0100.cpp, except that the functionality that is common
 *  between syncr<int> and lazy<int> has been placed in the base class coreturn<T>.
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

#include "print.h"
#include "tracker.h"

// -----------------------------------------------------------------

#include <coroutine>

template<typename T>
struct coreturn : private coroutine_tracker {
    struct promise;
    friend struct promise;

    using handle_type = std::coroutine_handle<promise>;

    coreturn(const coreturn&) = delete;

    coreturn(coreturn&& s)
        : coro(s.coro) {
        print("coreturn::coreturn(coreturn&& s)\n");
        s.coro = nullptr;
    }

    ~coreturn() {
        print("coreturn::~coreturn()\n");
        if (coro) {
            print("coreturn::~coreturn(): coro.done() = %d\n", coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    coreturn& operator = (const coreturn&) = delete;

    coreturn& operator = (coreturn&& s) {
        print("coreturn& coreturn::operator = (coreturn&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    struct promise : private promise_type_tracker {
        friend struct coreturn;

        promise() {
            print("coreturn::promise::promise()\n");
        }

        ~promise() {
            print("coreturn::promise::~promise()\n");
        }

        void return_value(T v) {
            print("auto coreturn::promise::return_value(T v)\n", v );
            value = v;
        }

        auto final_suspend() noexcept {
            print("auto coreturn::promise::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: coreturn::promise::unhandled_exception()\n", this);
            std::exit(1);
        }

    private:
        T value;
    };

protected:
    T get() {
        print("T coreturn::get()\n");
        return coro.promise().value;
    }

    coreturn(handle_type h)
        : coro(h) {
        print("coreturn::coreturn(handle_type h)\n");
    }

    handle_type coro;
};

// -----------------------------------------------------------------

template<typename T>
struct syncr : public coreturn<T> {
    using coreturn<T>::coreturn;
    using handle_type = typename coreturn<T>::handle_type;

    T get() {
        print("T syncr::get()\n");
        return coreturn<T>::get();
    }

    struct promise_type : public coreturn<T>::promise {
        auto get_return_object() {
            print("auto syncr::promise_type::get_return_object()\n");
            return syncr<T>{handle_type::from_promise(*this)};
        }
        auto initial_suspend() {
            print("auto syncr::promise_type::initial_suspend()\n");
            return std::suspend_never{};
        }
    };
};

// -----------------------------------------------------------------

template<typename T>
struct lazy : public coreturn<T> {
    using coreturn<T>::coreturn;
    using handle_type = typename coreturn<T>::handle_type;

    T get() {
        print("T lazy::get()\n");
        this->coro.resume();
        return coreturn<T>::get();
    }

    struct promise_type : public coreturn<T>::promise {
        auto get_return_object() {
            print("auto lazy::promise_type::get_return_object()\n");
            return lazy<T>{handle_type::from_promise(*this)};
        }
        auto initial_suspend() {
            print("auto lazy::promise_type::initial_suspend()\n");
            return std::suspend_always{};
        }
    };
};

// -----------------------------------------------------------------

syncr<int> answer1() {
    print("syncr<int> answer1()\n");
    co_return 42;
}

lazy<int> answer2() {
    print("lazy<int> answer2()\n");
    co_return 42;
}

// -----------------------------------------------------------------

void test_syncr() {
    print("auto a1 = answer1();\n");
    auto a1 = answer1();
    print("auto v = a1.get();\n");
    auto v = a1.get();
    print("The coroutine value is: %d\n", v);
}

void test_lazy() {
    print("auto a2 = answer2();\n");
    auto a2 = answer2();
    print("auto v = a2.get();\n");
    auto v = a2.get();
    print("The coroutine value is: %d\n", v);
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
