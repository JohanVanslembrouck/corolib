/**
 *  Filename: p0424.cpp
 *  Description:
 *         Illustrates the use of co_await.
 *
 *         Variant of p0420.cpp. See p0420.cpp for more details.
 *
 *         Uses a dedicated coroutine type (mini).
 *
 *         An global object of this type is co_awaited in coroutine5 and
 *         is resumed from main().
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

/*
 int my_first_coroutine(int i)
 {
     co_return 2 * i;
 }
*/

#include "print0.h"
#include "csemaphore.h"

//--------------------------------------------------------------

struct mini {

    std::coroutine_handle<> m_awaiting;

    void resume() {
        print("%p: mini::resume(): before m_awaiting.resume();\n", this);
        if (!m_awaiting.done())
            m_awaiting.resume();
        print("%p: mini::resume(): after m_awaiting.resume();\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(mini& mini_) :
                m_mini(mini_)
            {}

            bool await_ready() {
                print("%p: mini::await_ready(): return false\n", this);
                return false;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print("%p: mini::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_mini.m_awaiting = awaiting;
            }

            void await_resume() {
                print("%p: void mini::await_resume()\n", this);
            }

        private:
            mini& m_mini;
        };

        return awaiter{ *this };
    }
};

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
        //if (coro) coro.destroy();    // can throw exception when eager goes out of scope. FFS
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
        print("%p: eager::get()\n", this);
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }

#if 1
    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(eager& eager_) : 
                m_eager(eager_)
            {}

            bool await_ready() {
                bool ready = m_eager.coro.done();
                print("%p: eager::await_ready(): return %d\n", this, ready);
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
    bool await_ready() {
        bool ready = coro.done();
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

        auto return_value(T v) {
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

mini m;

eager<int> coroutine5() {
    print("coroutine5()\n");
    print("coroutine5(): co_await m;\n");
    co_await m;
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

    fprintf(stderr, "\n");
    print("coroutine3(): eager<int> a2 = coroutine4();\n");
    eager<int> a2 = coroutine4();
    print("coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    fprintf(stderr, "\n");
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

    fprintf(stderr, "\n");
    print("coroutine1(): co_return %d;\n", v+1);
    co_return v+1;
}

int main() {
    print("main(): eager<int> awa = coroutine1();\n");
    eager<int> awa = coroutine1();

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print("main(): before m.resume();\n");
    m.resume();
    print("main(): after m.resume();\n");

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print("main(): before m.resume();\n");
    m.resume();
    print("main(): after m.resume();\n");

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#if 0
    print("main(): before m.resume();\n");
    m.resume();
    print("main(): after m.resume();\n");

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print("main(): before m.resume();\n");
    m.resume();
    print("main(): after m.resume();\n");

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print("main(): before m.resume();\n");
    m.resume();
    print("main(): after m.resume();\n");

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif

    print("main(): int i = awa.get();\n");
    int i = awa.get();
    print("main(): i = %d\n", i);

    print("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print("main(): return 0;\n");
    return 0;
}
