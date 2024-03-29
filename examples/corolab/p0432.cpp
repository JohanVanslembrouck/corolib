/** 
 *  Filename: p0432.cpp
 *  Description:
 *      Illustrates the use of co_await.
 *
 *      Shows the use of a lazy coroutine type, i.e. a coroutine type
 *      that suspends at its beginning and returns control to the calling
 *      function or coroutine.
 *      The calling function or coroutine will resume the called coroutine
 *      from its get() function.
 *
 *      p0432.cpp is a variant of p0430.cpp.
 *      It initializes 5 lazy<int> objects with coroutines,
 *      in the order opposite to the order in which the coroutines will be called.
 *      The coroutines co_await the initialized lazy<int> objects.
 *      This approach is possible because of the lazy coroutine type.
 *      
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>
#include <coroutine>

#include "print.h"
#include "tracker.h"
#include "csemaphore.h"

// -------------------------------------------------------------

template<typename T>
struct lazy : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    lazy(const lazy& s) = delete;

    lazy(lazy&& s)
        : coro(s.coro) {
        print("%p: lazy::lazy(lazy&& s)\n", this);
        s.coro = nullptr;
    }

    ~lazy() {
        print("%p: lazy::~lazy()\n", this);
        if (coro) {
            print("%p: lazy::~lazy(): coro.done() = %d\n", this, coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    lazy& operator = (const lazy&) = delete;

    lazy& operator = (lazy&& s) {
        print("%p: lazy::lazy = (lazy&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("%p: lazy::get(): coro.resume();\n", this);
        coro.resume();

        print("%p: lazy::get(): spinlock\n", this);
        while (!coro.done())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        print("%p: lazy::get(): coro.promise().m_value;\n", this);
        return coro.promise().m_value;
    }

    bool await_ready() {
        const auto ready = coro.done();
        print("%p: lazy::await_ready(): return %d;\n", this, ready);
        return ready;
    }

#if 0
    void await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: lazy::await_suspend(...): coro.resume();\n", this);
        coro.resume();
        print("%p: lazy::await_suspend(...): awaiting.resume();\n", this);
        awaiting.resume();
        print("%p: lazy::await_suspend(...): return;\n", this);
    }
#else
    std::coroutine_handle<> await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: lazy::await_suspend(...): coro.resume();\n", this);
        coro.resume();
        print("%p: lazy::await_suspend(...): return awaiting;\n", this);
        return awaiting;
    }
#endif

    auto await_resume() {
        print("%p: lazy::await_resume()\n", this);
        const auto r = coro.promise().m_value;
        print("%p: lazy::await_resume(): coro.done() = %d\n", this, coro.done());
        return r;
    }

    struct promise_type : private promise_type_tracker {

        friend struct lazy;

        promise_type() : m_value(0) {
            print("%p: lazy::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: lazy::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print("%p: lazy::promise_type::return_value(T v)\n", this);
            this->m_value = v;
        }

        auto get_return_object() {
            print("%p: lazy::promise_type::get_return_object()\n", this);
            return lazy<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("%p: lazy::promise_type::initial_suspend()\n", this);
            return std::suspend_always{};
        }

        auto final_suspend() noexcept {
            print("%p: lazy::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: lazy::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        std::coroutine_handle<> m_awaiting;
    };

    lazy(handle_type h)
        : coro(h) {
        print("%p: lazy::lazy(handle_type h)\n", this);
    }

    handle_type coro;
};

// -------------------------------------------------------------

// Forward declarations
extern lazy<int> a5;
extern lazy<int> a4;
extern lazy<int> a3;
extern lazy<int> a2;

lazy<int> coroutine5() {
    print("coroutine5()\n");

#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    
    CSemaphore sema;

    std::thread thread1([&]() {
        print("thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("thread1: sema.signal();\n");
        sema.signal();
        });
    thread1.detach();

    print("coroutine5(): sema.wait();\n");
    sema.wait();
#endif

    int v = 42;

    print("coroutine5(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine4() {
    print("coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print("coroutine4(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine3() {
    print("coroutine3(): int v = co_await a4;\n");
    int v = co_await a4;
    print("coroutine3(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine2() {
    print("coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print("coroutine2(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine1() {
    print("coroutine1(): int v = co_await a2;\n");
    int v = co_await a2;
    print("coroutine1(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> a5 = coroutine5();
lazy<int> a4 = coroutine4();
lazy<int> a3 = coroutine3();
lazy<int> a2 = coroutine2();
lazy<int> a1 = coroutine1();

int main() {
    print("main(): int i = a1.get();\n");
    int i = a1.get();
    print("main(): i = %d\n", i);
    print("main(): return 0;\n");
    return 0;
}
