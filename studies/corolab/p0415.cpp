/** 
 *  Filename: p0415.cpp
 *  Description: 
 *        Illustrates the use of co_await.
 * 
 *        Based upon p0414.cpp.
 *        The difference with p0414.cpp is the use of a dedicated final_awaiter type with
 *        bool await_suspend(handle_type h) noexcept;
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
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

#define USE_FINAL_AWAITER 0

/**
Tracker
Without final_awaiter:

00:     cons    dest    diff    max     c>p     p>c     err
00: cor 7       7       0       6       0       0       0
00: pro 7       7       0       6       0       0

With final_awaiter:

HANGS!!!

*/

//--------------------------------------------------------------

template<typename T>
struct eager : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    eager(const eager& s) = delete;

    eager(eager&& s)
        : coro(s.coro) {
        print("%p: eager::eager(eager&& s)\n", this);
        s.coro = nullptr;
    }

    ~eager() {
        print("%p: eager::~eager()\n", this);
        if (coro) {
            print("%p: eager::~eager(): coro.done() = %d\n", this, coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    eager(handle_type h)
        : coro(h) {
        print("%p: eager::eager(handle_type h)\n", this);
    }

    eager& operator = (const eager&) = delete;

    eager& operator = (eager&& s) {
        print("%p: eager::eager = (eager&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("%p: eager::get(); coro.done() = %d, coro.promise().m_ready = %d\n", this, coro.done(), coro.promise().m_ready);
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }
    
    // Alternative 1: define operator co_await and an awaiter type
    // that defines await_ready(), await_suspend() and await_resume().
    
    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(eager& eager_) : 
                m_eager(eager_)
            {}

            bool await_ready() {
                const bool ready = m_eager.coro.done();
                print("%p: eager::await_ready(): return %d;\n", this, ready);
                return ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print("%p: eager::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_eager.coro.promise().m_awaiting = awaiting;
            }

            T await_resume() {
                print("%p: eager::await_resume()\n", this);
                const T r = m_eager.coro.promise().m_value;
                return r;
            }

        private:
            eager& m_eager;
        };

        return awaiter{*this};
    }

    struct promise_type : private promise_type_tracker {

        friend struct eager;

        promise_type() :
            m_value{},
            m_ready{false},
            m_awaiting(nullptr),
            m_wait_for_signal(false) {
            print("%p: eager::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: eager::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print("%p: eager::promise_type::return_value(T v): begin\n", this);
            m_ready = true;
            m_value = v;
            if (m_awaiting) {
                print("%p: eager::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print("%p: eager::promise_type::return_value(T v): after m_awaiting.resume();\n", this);
            }
            if (m_wait_for_signal) {
                print("%p: eager::promise_type::return_value(T v): before m_sema.signal();\n", this);
                m_sema.signal();
                print("%p: eager::promise_type::return_value(T v): after m_sema.signal();\n", this);
            }
            print("%p: eager::promise_type::return_value(T v): end\n", this);
        }

        auto get_return_object() {
            print("%p: eager::promise_type::get_return_object()\n", this);
            return eager<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("%p: eager::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

        struct final_awaiter {
            bool await_ready() const noexcept {
                print("%p: eager::promise_type::final_awaiter::await_ready()\n", this);
                return false;
            }

            bool await_suspend(handle_type h) noexcept {
                print("%p: eager::promise_type::final_awaiter::await_suspend()\n", this);
                promise_type& promise = h.promise();

                if (promise.m_ready) {
                    print("%p: eager::promise_type::final_awaiter::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    print("%p: eager::promise_type::final_awaiter::await_suspend(): m_value = %d\n", this, promise.m_value);
                }
                return !promise.m_ready;
            }

            void await_resume() noexcept {
                print("%p: eager::promise_type::final_awaiter::await_resume()\n", this);
            }
        };

        auto final_suspend() noexcept {
            print("%p: eager::promise_type::final_suspend()\n", this);
#if USE_FINAL_AWAITER
            return final_awaiter{};
#else
            return std::suspend_always{};
#endif
        }

        void unhandled_exception() {
            print("%p: eager::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        bool m_ready;
        CSemaphore m_sema;
        std::coroutine_handle<> m_awaiting;
        bool m_wait_for_signal;
    };

    handle_type coro;
};

//--------------------------------------------------------------

eager<int> coroutine5() {
    print("coroutine5(): int v = 42;\n");
    int v = 42;
    print("coroutine5(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine4() {
    print("coroutine4(): eager<int> a = coroutine5();\n");
    eager<int> a = coroutine5();
    print("coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine4(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine3() {
    print("coroutine3(): eager<int> a1 = coroutine4();\n");
    eager<int> a1 = coroutine4();
    print("coroutine3(): int v = co_await a1;\n");
    int v1 = co_await a1;

    print(); print("coroutine3(): eager<int> a2 = coroutine4();\n");
    eager<int> a2 = coroutine4();
    print("coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    print("coroutine3(): co_return %d;\n", v1 + v2 + 1);
    co_return v1+v2+1;
}

eager<int> coroutine2() {
    print("coroutine2(): eager<int> a = coroutine3();\n");
    eager<int> a = coroutine3();
    print("coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine2(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine1() {
    print("coroutine1(): eager<int> a = coroutine2();\n");
    eager<int> a = coroutine2();
    print("coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine1(): co_return %d;\n", v+1);
    co_return v+1;
}

/**
 * Because main() cannot be a coroutine (it cannot return a coroutine type),
 * it cannot use co_await. Instead it calls get() on the coroutine object
 * returned from coroutine1().
 */
int main() {
    print("main(): eager<int> awa = coroutine1();\n");
    eager<int> awa = coroutine1();
    print("main(): int i = awa.get();\n");
    int i = awa.get();
    print("main(): i = %d\n", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    print("main(): return 0;\n");
    return 0;
}
