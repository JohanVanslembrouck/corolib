/**
 *  Filename: p0120.cpp
 *  Description:
 *  Defines two coroutine types for coroutines that use co_return.
 *  Type 1: sync<T>: eager coroutine type: the coroutine starts executing upon entry.
 *  Type 2: lazy<T>: lazy coroutine type: upon entry, it immediately returns control
 *                   to its calling function/coroutine that is responsible for resuming
 *                   the lazy coroutine.
 *  p0120.cpp uses the same coroutine definitions as p0110.cpp,
 *  but uses a "chain" of coroutine calls.
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/A%20more%20realistic%20coroutine
 */

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>

// -----------------------------------------------------------------


/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */

uint64_t threadids[128];

int get_thread_number64(uint64_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

int get_thread_number32(uint32_t id)
{
    for (int i = 0; i < 128; i++)
    {
        if (threadids[i] == id)
            return i;
        if (threadids[i] == 0) {
            threadids[i] = id;
            return i;
        }
    }
    return -1;
}

uint64_t get_thread_id()
{
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t)(*ptr);
}

void print(const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    fprintf(stderr, "%02d: %s", threadid, msg);
}


// -----------------------------------------------------------------

#include <experimental/resumable>

template<typename T>
struct coreturn {
    struct promise;
    friend struct promise;

    using handle_type = std::experimental::coroutine_handle<promise>;

    coreturn(const coreturn&) = delete;

    coreturn(coreturn&& s)
        : coro(s.coro) {
        print("coreturn::coreturn(coreturn&& s)\n");
        s.coro = nullptr;
    }

    ~coreturn() {
        print("~coreturn::coreturn()\n");
        if (coro) coro.destroy();
    }

    coreturn& operator = (const coreturn&) = delete;

    coreturn& operator = (coreturn&& s) {
        print("coreturn& coreturn::operator = (coreturn&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    struct promise {
        friend struct coreturn;

        promise() {
            print("coreturn::promise::promise()\n");
        }

        ~promise() {
            print("~coreturn::promise::promise()\n");
        }

        void return_value(T v) {
            print("auto coreturn::promise::return_value(T v)\n", v );
            value = v;
        }

        auto final_suspend() {
            print("auto coreturn::promise::final_suspend()\n");
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
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
struct sync : public coreturn<T> {
    using coreturn<T>::coreturn;
    using handle_type = typename coreturn<T>::handle_type;

    T get() {
        print("T sync::get()\n");
        return coreturn<T>::get();
    }

    struct promise_type : public coreturn<T>::promise {
        auto get_return_object() {
            print("auto sync::promise_type::get_return_object()\n");
            return sync<T>{handle_type::from_promise(*this)};
        }
        auto initial_suspend() {
            print("auto sync::promise_type::initial_suspend()\n");
            return std::experimental::suspend_never{};
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
            return std::experimental::suspend_always{};
        }
    };
};

// -----------------------------------------------------------------

sync<int> answer5() {
    print("sync<int> answer1()\n");
    co_return 42;
}

sync<int> answer4() {
    print("answer4(): sync<int> a5 = answer5();\n");
    sync<int> a5 = answer5();
    print("answer4(): int a5 = a5.get();\n");
    int i5 = a5.get();
    print("answer3(): co_return i5 + 1;\n");
    co_return i5 + 1;
}

sync<int> answer3() {
    print("answer3(): sync<int> a4 = answer4();\n");
    sync<int> a4 = answer4();
    print("answer3(): int a4 = a4.get();\n");
    int i4= a4.get();
    print("answer3(): co_return i4 + 1;\n");
    co_return i4 + 1;
}

sync<int> answer2() {
    print("answer2(): sync<int> a3 = answer3();\n");
    sync<int> a3 = answer3();
    print("answer2(): int i3 = a3.get();\n");
    int i3 = a3.get();
    print("answer2(): co_return i3 + 1;\n");
    co_return i3 + 1;
}

sync<int> answer1() {
    print("answer1(): sync<int> a2 = answer2();\n");
    sync<int> a2 = answer2();
    print("answer1(): int i2 = a2.get();\n");
    int i2 = a2.get();
    print("answer1(): co_return i2 + 42;\n");
    co_return i2 + 1;
}

void test_sync() {
    print("test_sync(): sync<int> a1 = answer1();\n");
    sync<int> a1 = answer1();
    print("test_sync(): int v = a1.get();\n");
    int v = a1.get();
    print("test_sync(): The coroutine value is: %d\n", v);
}

// -----------------------------------------------------------------

lazy<int> answer15() {
    print("lazy<int> answer15()\n");
    co_return 42;
}

lazy<int> answer14() {
    print("answer14(): lazy<int> a15 = answer15();\n");
    lazy<int> a15 = answer15();
    print("answer14(): int a5 = a15.get();\n");
    int i15 = a15.get();
    print("answer13(): co_return i15 + 1;\n");
    co_return i15 + 1;
}

lazy<int> answer13() {
    print("answer13(): lazy<int> a14 = answer14();\n");
    lazy<int> a14 = answer14();
    print("answer13(): int a4 = a14.get();\n");
    int i14 = a14.get();
    print("answer13(): co_return i14 + 1;\n");
    co_return i14 + 1;
}

lazy<int> answer12() {
    print("answer12(): lazy<int> a13 = answer13();\n");
    lazy<int> a13 = answer13();
    print("answer12(): int i13 = a13.get();\n");
    int i13 = a13.get();
    print("answer12(): co_return i13 + 1;\n");
    co_return i13 + 1;
}

lazy<int> answer11() {
    print("answer11(): lazy<int> a12 = answer12();\n");
    lazy<int> a12 = answer12();
    print("answer11(): int i12 = a12.get();\n");
    int i12 = a12.get();
    print("answer11(): co_return i12 + 42;\n");
    co_return i12 + 1;
}

void test_lazy() {
    print("test_lazy(): lazy<int> a11 = answer11();\n");
    lazy<int> a11 = answer11();
    print("test_lazy(): int v = a11.get();\n");
    int v = a11.get();
    print("test_lazy(): The coroutine value is: %d\n", v);
}

// -----------------------------------------------------------------

int main() {
    print("test_sync();\n");
    test_sync();
    fprintf(stderr, "\n");
    print("test_lazy();\n");
    test_lazy();
    return 0;
}

