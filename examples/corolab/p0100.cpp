/**
 *  Filename: p0100.cpp
 *  Description:
 *  Defines two coroutine types for coroutines that use co_return.
 *  Type 1: sync<T>: eager coroutine type: the coroutine starts executing upon entry.
 *  Type 2: lazy<T>: lazy coroutine type: upon entry, it immediately returns control
 *                   to its calling function/coroutine that is responsible for resuming
 *                   the lazy coroutine.
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

/**
 * Eager coroutine type.
 *
 */

template<typename T>
struct sync {
    struct promise_type;
    friend struct promise_type;

    using handle_type = std::experimental::coroutine_handle<promise_type>;

    sync(const sync&) = delete;

    sync(sync&& s)
        : coro(s.coro) {
        print("sync::sync(sync&& s)\n");
        s.coro = nullptr;
    }

    ~sync() {
        print("~sync::sync()\n");
        if (coro) coro.destroy();
    }

    sync& operator = (const sync&) = delete;

    sync& operator = (sync&& s) {
        print("sync& sync::operator = (sync&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }
    
    T get() {
        print("T sync::get()\n");
        return coro.promise().value;
    }

    struct promise_type {
        friend struct sync;

        promise_type() {
            print("sync::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print("~sync::promise_type::promise_type()\n");
        }

        auto get_return_object() {
            print("auto sync::promise_type::get_return_object()\n");
            return sync<T>{handle_type::from_promise(*this)};
        }
        
        void return_value(T v) {
            print("auto sync::promise_type::return_value(T v)\n", v );
            value = v;
        }

        auto initial_suspend() {
            print("auto sync::promise_type::initial_suspend()\n");
            return std::experimental::suspend_never{};
        }
            
        auto final_suspend() {
            print("auto sync::promise_type::final_suspend()\n");
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            std::exit(1);
        }

    private:
        T value;
    };

public:
    sync(handle_type h)
        : coro(h) {
        print("sync::sync(handle_type h)\n");
    }
    
    handle_type coro;
};

// -----------------------------------------------------------------

template<typename T>
struct lazy {
    struct promise_type;
    friend struct promise_type;

    using handle_type = std::experimental::coroutine_handle<promise_type>;

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
        
        auto return_value(T v) {
            print("auto lazy::promise_type::return_value(T v)\n", v );
            value = v;
            return std::experimental::suspend_never{};
        }

        auto initial_suspend() {
            print("auto lazy::promise_type::initial_suspend()\n");
            return std::experimental::suspend_always{};
        }
        
        auto final_suspend() {
            print("auto lazy::promise_type::final_suspend()\n");
            return std::experimental::suspend_always{};
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

sync<int> answer1() {
    co_return 42;
}

into

sync<int> answer1() {
    sync<int>::promise_type p;
    auto task = p.initial_suspend();
    sync<int> ret = p.get_return_object();
    co_await task;

    //co_return 42;
    p.return_value(42);
final_suspend:
    co_await p.final_suspend();
}

The numbers 1 till 4 show the control flow between
the calling function test_sync() and the called coroutine answer1().
The numbers are followed with trace output.

sync<T> behaves more or less as a normal function, except
that it "dies" later than a normal function, namely
when it terminates after the final suspend point.
 */

sync<int> answer1() {
    // 2
    // 00: sync::promise_type::promise_type()
    // 00: auto sync::promise_type::initial_suspend()
    // 00: auto sync::promise_type::get_return_object()
    // 00: sync::sync(handle_type h)

    print("sync<int> answer1()\n");
    
    co_return 42;
    
    // 00: auto sync::promise_type::return_value(T v)
    // 00: auto sync::promise_type::final_suspend()
    
    // 4
    // 00: ~sync::promise_type::promise_type()
}

void test_sync() {
    // 1
    print("sync<int> a1 = answer1();\n");
    sync<int> a1 = answer1();
    
    // 3
    print("int v = a1.get();\n");
    int v = a1.get();
    print("The coroutine value is: %d\n", v);
    
    // 00: ~sync::sync()
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
    print("test_sync();\n");
    test_sync();
    fprintf(stderr, "\n");
    print("test_lazy();\n");
    test_lazy();
    return 0;
}

