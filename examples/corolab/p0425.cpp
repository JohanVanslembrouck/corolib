/**
 *  Filename: p0425.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *        Based upon p0424.cpp.
 *        The difference with p0424.cpp is the use of a dedicated final_awaiter type with
 *        std::coroutine_handle<> await_suspend(handle_type h) noexcept;
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

#define USE_FINAL_AWAITER 1

/**
Tracker
Without final_awaiter:

00:     cons    dest    diff    max     c>p     p>c     err
00: cor 7       7       0       7       0       0       0
00: pro 7       2       5       7       0       0

With final_awaiter:

00:     cons    dest    diff    max     c>p     p>c     err
00: cor 7       7       0       6       0       0       0
00: pro 7       7       0       6       0       0
*/

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
struct eager : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    eager(const eager& s) = delete;

    eager(eager&& s)
        : coro(s.coro)
        , m_value(s.m_value)
        , m_ready(s.m_ready) {
        print("%p: eager::eager(eager&& s)\n", this);
        s.coro = nullptr;
        s.m_value = {};
        s.ready = false;
    }

    ~eager() {
        print("%p: eager::~eager()\n", this);
        coro.promise().link_coroutine_object(nullptr);
        if (coro) {
            print("%p: eager::~eager(): coro.done() = %d\n", this, coro.done());
            if (coro.done())        // Do not destroy if not yet done
                coro.destroy();
        }
    }

    eager(handle_type h)
        : coro(h)
        , m_value{}
        , m_ready{ false } {
        print("%p: eager::eager(handle_type h)\n", this);
        coro.promise().link_coroutine_object(this);
    }

    eager& operator = (const eager&) = delete;

    eager& operator = (eager&& s) {
        print("%p: eager::eager = (eager&& s)\n", this);
        coro = s.coro;
        m_value = s.m_value;
        m_ready = s.m_ready;
        s.coro = nullptr;
        s.m_value = {};
        s.m_ready = false;
        return *this;
    }

    T get() {
        print("%p: eager::get()\n", this);
        if (m_ready)
            return m_value;
        else
            return {};
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(eager& eager_) : 
                m_eager(eager_)
            {}

            bool await_ready() {
                bool ready = m_eager.m_ready; // m_eager.coro.done();
                print("%p: eager::await_ready(): return %d\n", this, ready);
                return ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print("%p: eager::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_eager.coro.promise().m_awaiting = awaiting;
            }

            T await_resume() {
                print("%p: eager::await_resume()\n", this);
                const T r = m_eager.m_value;
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
            m_awaiting(nullptr),
            m_coroutine_object(nullptr) {
            print("%p: eager::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: eager::promise_type::~promise_type()\n", this);
        }

        auto return_value(T v) {
            print("%p: eager::promise_type::return_value(T v): begin\n", this);
            if (m_coroutine_object) {
                m_coroutine_object->m_value = v;
                m_coroutine_object->m_ready = true;
            }
            if (m_awaiting) {
                // Do nothing
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

            std::coroutine_handle<> await_suspend(handle_type h) noexcept {
                print("%p: eager::promise_type::final_awaiter::await_suspend()\n", this);

                if (h.promise().m_awaiting)
                    return h.promise().m_awaiting;
                else
                    return std::noop_coroutine();
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

        void link_coroutine_object(eager* coroutine_object)
        {
            m_coroutine_object = coroutine_object;
            print("%p: eager::promise_type::link_coroutine_object(%p)\n", this, m_coroutine_object);
        }

    private:
        std::coroutine_handle<> m_awaiting;
        eager* m_coroutine_object;
    };

    handle_type coro;
    T m_value;
    bool m_ready;
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
