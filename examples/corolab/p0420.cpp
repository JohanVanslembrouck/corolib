/** 
 *  Filename: p0420.cpp
 *  Description: 
 *        Illustrates the use of co_await.
 *
 *        Shows the use of an eager coroutine tyoe, i.e. a coroutine type
 *        that starts executing and only suspends when it encounters 
 *        a co_await or co_return statement.
 *        The coroutine is resumed from the coroutine or function it called.
 *        To make this possible, 
 *            void await_suspend(std::coroutine_handle<> awaiting)
 *        stores the coroutine handle of the current coroutine in
 *        in the promise of the called coroutine.
 *        When the called coroutine returns by calling co_return, which is
 *        translated by the compiler into return_value(), return_value resumes
 *        the calling coroutine.
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

#include "print0.h"
#include "csemaphore.h"

//--------------------------------------------------------------

template<typename T>
struct eager {

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
        print("%p: eager::get(); coro.done() = %d\n", this, coro.done());
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }
    
#if 1
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
#else
    // Alternative 2: define await_ready(), await_suspend() and await_resume()
    // in the coroutine type.
    
    bool await_ready() {
        const bool ready = coro.done();
        print("%p: eager::await_ready(): return %d;\n", this, ready);
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: eager::await_suspend(std::coroutine_handle<> awaiting)\n", this);
        coro.promise().m_awaiting = awaiting;
    }

    T await_resume() {
        print("%p: eager::await_resume()\n", this);
        const T r = coro.promise().m_value;
        return r;
    }
#endif

    struct promise_type  {

        friend struct eager;

        promise_type() :
            m_value{},
            m_awaiting(nullptr),
            m_wait_for_signal(false) {
            print("%p: eager::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: eager::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print("%p: eager::promise_type::return_value(T v): begin\n", this);
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

        auto final_suspend() noexcept {
            print("%p: eager::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: eager::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        CSemaphore m_sema;
        std::coroutine_handle<> m_awaiting;
        bool m_wait_for_signal;
    };

    handle_type coro;
};

//--------------------------------------------------------------
/**
 * coroutine5() starts a thread and then co_wait's a coroutine object
 * created at its beginning.
 * The thread starts a timer and resumes coroutine5() when the timer expires.
 * This implementation uses implementation details from eager<T> and
 * is therefore more a hack.
 * See p0422.cpp for a cleaner solution, but uses an extra coroutine type.
 */

eager<int> coroutine5() {
    print("coroutine5()\n");
    
#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    
    eager<int>::promise_type p;
    p.initial_suspend();    // without this statement, m_eager.coro.done() in await_ready() returns 1 instead of 0
    eager<int> a = p.get_return_object();

    std::thread thread1([&]() {
        print("thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        fprintf(stderr, "\n");
        print("thread1: p.set_return_value(42);\n");
        p.return_value(42);
        p.final_suspend();
        });
    thread1.detach();

    print("coroutine5(): int si = co_await a;\n");
    int v = co_await a;
#else
    int v = 42;
#endif
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

    fprintf(stderr, "\n");
    print("coroutine3(): eager<int> a2 = coroutine4();\n");
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
