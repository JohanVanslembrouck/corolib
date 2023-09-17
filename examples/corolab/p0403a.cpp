/**
 *  Filename: p0403a.cpp
 *  Description:
 *  Illustrates the use of and the translation of co_await.
 *  See p0403at.cpp for snippets of translated code.
 *
 *  Uses void await_suspend(std::coroutine_handle<> awaiting)
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <coroutine>

using namespace std;

#include "print.h"
#include "tracker.h"


// -----------------------------------------------------------------

struct auto_reset_event {

    std::coroutine_handle<> m_awaiting;

    auto_reset_event()
        : m_awaiting(nullptr)
        , m_ready(false) {
        print(PRI2, "auto_reset_event::auto_reset_event()\n");
    }

    auto_reset_event(const auto_reset_event&) = delete;
    auto_reset_event& operator = (const auto_reset_event&) = delete;

    auto_reset_event(auto_reset_event&& s) noexcept
        : m_awaiting(s.m_awaiting)
        , m_ready(s.m_ready) {
        print(PRI2, "auto_reset_event::auto_reset_event(auto_reset_event&& s)\n");
        s.m_awaiting = nullptr;
        s.m_ready = false;
    }

    auto_reset_event& operator = (auto_reset_event&& s) noexcept {
        print(PRI2, "auto_reset_event::operator = (auto_reset_event&& s)\n");
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        return *this;
    }

    void resume() {
        print(PRI2, "auto_reset_event::resume()\n");
        m_ready = true;
        if (m_awaiting && !m_awaiting.done())
            m_awaiting.resume();
    }

    auto operator co_await() noexcept
    {
        struct awaiter
        {
            awaiter(auto_reset_event& are_) : m_are(are_) {
                print(PRI2, "auto_reset_event::awaiter::awaiter(auto_reset_event& are_)\n");
            }

            bool await_ready() {
                print(PRI2, "auto_reset_event::awaiter::await_ready()\n");
                return m_are.m_ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print(PRI2, "auto_reset_event::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
                m_are.m_awaiting = awaiting;
            }

            void await_resume() {
                print(PRI2, "auto_reset_event::awaiter::await_resume()\n");
                m_are.m_ready = false;
            }

        private:
            auto_reset_event& m_are;
        };

        return awaiter{ *this };
    }

    bool m_ready;
};

// -----------------------------------------------------------------

template <typename T>
struct awaitable : private coroutine_tracker
{
    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    awaitable(coro_handle coroutine)
        : m_coroutine(coroutine) {
        print(PRI2, "awaitable::awaitable(coro_handle coroutine)\n");
    }

    ~awaitable() {
        print(PRI2, "awaitable::~awaitable()\n");
        if (m_coroutine) {
            print(PRI2, "awaitable::~awaitable(): m_coroutine.done() = %d\n", m_coroutine.done());
            if (m_coroutine.done())
                m_coroutine.destroy();
        }
    }

    awaitable() = default;
    awaitable(awaitable const&) = delete;
    awaitable& operator= (awaitable const&) = delete;

    awaitable(awaitable&& other)
        : m_coroutine(other.m_coroutine) {
        print(PRI2, "awaitable::awaitable(awaitable&& other)\n");
        other.m_coroutine = nullptr;
    }

    awaitable& operator= (awaitable&& other) {
        print(PRI2, "awaitable::operator= (awaitable&& other)\n");
        if (&other != this) {
            m_coroutine = other.m_coroutine;
            other.m_coroutine = nullptr;
        }
    }

    T get() {
        print(PRI2, "awaitable::get()\n");
        if (m_coroutine)
            return m_coroutine.promise().m_value;
        return -1;
    }

    // defined in template<typename T> struct awaitable
    auto operator co_await() noexcept {
        struct awaiter {
            awaiter(awaitable& awaitable_) : m_awaitable(awaitable_) {
                print(PRI2, "awaitable::awaiter::awaiter(awaitable& awaitable_)\n");
            }

            bool await_ready() noexcept { 
                print(PRI2, "awaitable::awaiter::await_ready()\n");
                return m_awaitable.m_coroutine.promise().m_ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) noexcept {
                print(PRI2, "awaitable::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
                m_awaitable.m_coroutine.promise().m_awaiting = awaiting;
            }

            T await_resume() noexcept {
                print(PRI2, "awaitable::awaiter::await_resume()\n");
                return m_awaitable.m_coroutine.promise().m_value;
            }

            awaitable& m_awaitable;
        };
        return awaiter{ *this };
    }

    // defined in template<typename T> struct awaitable
    struct promise_type : private promise_type_tracker
    {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type() 
            : m_value(0)
            , m_ready(false)
            , m_awaiting(nullptr) {
            print(PRI2, "awaitable::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI2, "awaitable::promise_type::~promise_type()\n");
        }

        auto get_return_object() { 
            print(PRI2, "awaitable::promise_type::get_return_object()\n");
            return coro_handle::from_promise(*this);
        }

        static awaitable get_return_object_on_allocation_failure() {
            print(PRI2, "awaitable::promise_type::get_return_object_on_allocation_failure()\n");
            throw std::bad_alloc();
        }

        auto initial_suspend() {
            print(PRI2, "awaitable::promise_type::initial_suspend()\n"); 
            return std::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "awaitable::promise_type::final_suspend()\n"); 
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "awaitable::promise_type::unhandled_exception()\n");
            std::terminate();
        }

        void return_value(T v) {
            print(PRI2, "awaitable::promise_type::return_value(T v)\n");
            m_value = v;
            m_ready = true;
            if (m_awaiting) m_awaiting.resume();
        }
        
        T m_value;
        bool m_ready;
        std::coroutine_handle<> m_awaiting;
    };

    coro_handle m_coroutine = nullptr;
};

// -----------------------------------------------------------------

auto_reset_event are1;

awaitable<int> coroutine2() {
    print(PRI1, "coroutine2(): co_await are1;\n");
    co_await are1;
    print(PRI1, "coroutine2(): co_return 42;\n");
    co_return 42;
}

awaitable<int> coroutine1() {
    print(PRI1, "coroutine1(): int i = co_await coroutine2();\n");
    int i = co_await coroutine2();
    print(PRI1, "coroutine1(): co_return 42 + i;\n");
    co_return 42 + i;
}

int main() {
    priority = 0x0F;
    print(PRI1, "main(): awaitable the_coroutine1 = coroutine1();\n");
    awaitable<int> the_coroutine1 = coroutine1();
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
    print(PRI1, "main(): int i = the_coroutine1.get();\n");
    int i = the_coroutine1.get();
    print(PRI1, "main(): i = %d\n", i);
    return 0;
}
