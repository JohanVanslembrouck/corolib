/**
 *  Filename: p0435.cpp
 *  Description:
 *      Illustrates the use of co_await.
 *
 *      Shows the use of a lazy coroutine type, i.e. a coroutine type
 *      that suspends at its beginning and returns control to the calling
 *      function or coroutine.
 *      The calling function or coroutine will resume the called coroutine
 *      from its get() function.
 *
 *      p0436.cpp is a variant of p0435.cpp.
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

#include <atomic>

class single_consumer_event
{
public:
    single_consumer_event(bool initiallySet = false) noexcept
        : m_state(initiallySet ? state::set : state::not_set)
    {}

    bool is_set() const noexcept
    {
        return m_state.load(std::memory_order_acquire) == state::set;
    }

    void set()
    {
        const state oldState = m_state.exchange(state::set, std::memory_order_acq_rel);
        if (oldState == state::not_set_consumer_waiting)
        {
            m_awaiter.resume();
        }
    }

    void reset() noexcept
    {
        state oldState = state::set;
        m_state.compare_exchange_strong(oldState, state::not_set, std::memory_order_relaxed);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(single_consumer_event& event) : m_event(event) {}

            bool await_ready() const noexcept
            {
                return m_event.is_set();
            }

            bool await_suspend(std::coroutine_handle<> awaiter)
            {
                m_event.m_awaiter = awaiter;

                state oldState = state::not_set;
                return m_event.m_state.compare_exchange_strong(
                    oldState,
                    state::not_set_consumer_waiting,
                    std::memory_order_release,
                    std::memory_order_acquire);
            }

            void await_resume() noexcept {}

        private:
            single_consumer_event& m_event;

        };
        return awaiter{ *this };
    }

private:

    enum class state
    {
        not_set,
        not_set_consumer_waiting,
        set
    };

    std::atomic<state> m_state;
    std::coroutine_handle<> m_awaiter;
};

template<typename T>
struct lazy : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    lazy(const lazy& s) = delete;

    lazy(lazy&& s)
        : coro(s.coro) {
        print("lazy::lazy(lazy&& s)\n");
        s.coro = nullptr;
    }

    ~lazy() {
        print("lazy::~lazy()\n");
        if (coro) {
            print("%p: lazy::~lazy(): coro.done() = %d\n", this, coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    lazy& operator = (const lazy&) = delete;

    lazy& operator = (lazy&& s) {
        print("lazy::lazy = (lazy&& s)\n");
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("lazy::get(): coro.resume();\n");
        coro.resume();

        print("lazy::get(): spinlock\n");
        //// No other activity anymore when entering the loop if we don't call coro.resume() first
        while (!coro.promise().set)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        print("lazy::get(): coro.promise().value;\n");
        return coro.promise().value;
    }

    bool await_ready() {
        const auto ready = coro.promise().set; // coro.done();
        print("lazy::await_ready(): return %d;\n", ready);
        //const auto ready = this->coro.done();
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print("lazy::await_suspend(...): coro.resume();\n");
        //coro.promise().m_awaiting = awaiting;
        coro.resume();
        print("lazy::await_suspend(...): awaiting.resume();\n");
        awaiting.resume();
    }

    auto await_resume() {
        print("lazy::await_resume()\n");
        const auto r = coro.promise().value;
        print("lazy::await_resume(): coro.promise().set = %d\n", coro.promise().set);
        return r;
    }

    struct promise_type : private promise_type_tracker {

        friend struct lazy;

        promise_type() : set(false) {
            print("lazy::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print("lazy::promise_type::~promise_type()\n");
        }

        auto return_value(T v) {
            print("lazy::promise_type::return_value(T v)\n");
            this->value = v;
            this->set = true;
            //this->m_awaiting.resume();
        }

        auto get_return_object() {
            print("lazy::promise_type::get_return_object()\n");
            return lazy<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("lazy::promise_type::initial_suspend()\n");
            return std::suspend_always{};
        }

        auto final_suspend() noexcept {
            print("lazy::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("lazy::promise_type::unhandled_exception()\n");
            std::exit(1);
        }

    private:
        T value;
        bool set;
        std::coroutine_handle<> m_awaiting;
    };

    lazy(handle_type h)
        : coro(h) {
        print("lazy::lazy(handle_type h)\n");
    }

    handle_type coro;
};

// Forward declarations
extern lazy<int> a5;
extern lazy<int> a4;
extern lazy<int> a3;
extern lazy<int> a2;

lazy<int> coroutine5() {
    print("coroutine5()\n");

#if 0
    lazy<int>::promise_type p;
    lazy<int> a = p.get_return_object();

    std::thread thread1([&]() {
        print("thread1: std::this_thread::sleep_for(std::chrono::milliseconds(3000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        print("thread1: p.set_return_value(42);\n");
        p.return_value(42);
        });
    thread1.join();

    print("coroutine5(): int si = co_await a;\n");
    int v = co_await a;
#else
    single_consumer_event try5;

    std::thread thread1([&try5]() {
        print("answer4(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("answer4(): thread1: try5.set();\n");
        try5.set();
        print("answer4(): thread1: return;\n");
        });
    thread1.join();

    print("answer4(): co_await try5;\n");
    int v = 42;
    co_await try5;
#endif
    
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
