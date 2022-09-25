/**
 *  Filename: p0406.cpp
 *  Description:
 *  Defines a coroutine type awaitable2 that does not make the coroutine
 *  to return an object to its calling function/coroutine.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <coroutine>

using namespace std;

const int priority = 0x0F;

#include "print.h"

// -----------------------------------------------------------------

template <typename T>
struct awaitable
{
    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    awaitable(coro_handle coroutine)
        : m_coroutine(coroutine) {
        print(PRI2, "awaitable::awaitable(coro_handle coroutine)\n");
    }

    ~awaitable() {
        print(PRI2, "awaitable::~awaitable()\n");
        if (m_coroutine)
            m_coroutine.destroy(); 
    }

    bool resume() {
        print(PRI2, "awaitable::resume()\n");
        if (!m_coroutine.done())
            m_coroutine.resume();
        return !m_coroutine.done();
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
                return false;
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
    struct promise_type
    {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type() 
            : m_value(0)
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

        void return_value(int v) {
            print(PRI2, "awaitable::promise_type::return_value(int v)\n");
            m_value = v;
            if (m_awaiting) m_awaiting.resume();
        }
        
        T m_value;
        std::coroutine_handle<> m_awaiting;
    };

    coro_handle m_coroutine = nullptr;
};

// -----------------------------------------------------------------

template <typename T>
struct awaitable2
{
    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    awaitable2(coro_handle coroutine)
        : m_coroutine(coroutine) {
        print(PRI2, "awaitable2::awaitable2(coro_handle coroutine)\n");
    }

    ~awaitable2() {
        print(PRI2, "awaitable2::~awaitable2()\n");
        if (m_coroutine)
            m_coroutine.destroy();
    }

    awaitable2() = default;
    awaitable2(awaitable2 const&) = delete;
    awaitable2& operator= (awaitable2 const&) = delete;

    awaitable2(awaitable2&& other)
        : m_coroutine(other.m_coroutine) {
        print(PRI2, "awaitable2::awaitable2(awaitable2&& other)\n");
        other.m_coroutine = nullptr;
    }

    awaitable2& operator= (awaitable2&& other) {
        print(PRI2, "awaitable2::operator= (awaitable2&& other)\n");
        if (&other != this) {
            m_coroutine = other.m_coroutine;
            other.m_coroutine = nullptr;
        }
    }

    T get() {
        print(PRI2, "awaitable2::get()\n");
        if (m_coroutine)
            return m_coroutine.promise().m_value;
        return -1;
    }

    // defined in template<typename T> struct awaitable
    auto operator co_await() noexcept {
        struct awaiter {
            awaiter(awaitable2& awaitable_) : m_awaitable(awaitable_) {
                print(PRI2, "awaitable2::awaiter::awaiter(awaitable2& awaitable_)\n");
            }

            bool await_ready() noexcept {
                print(PRI2, "awaitable2::awaiter::await_ready()\n");
                return true;
            }

            void await_suspend(std::coroutine_handle<> awaiting) noexcept {
                print(PRI2, "awaitable2::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
                m_awaitable.m_coroutine.promise().m_awaiting = awaiting;
            }

            T await_resume() noexcept {
                print(PRI2, "awaitable2::awaiter::await_resume()\n");
                return m_awaitable.m_coroutine.promise().m_value;
            }

            awaitable2& m_awaitable;
        };
        return awaiter{ *this };
    }

    // defined in template<typename T> struct awaitable
    struct promise_type
    {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type()
            : m_value(0)
            , m_awaiting(nullptr) {
            print(PRI2, "awawaitable2aitable::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI2, "awaitable2::promise_type::~promise_type()\n");
        }
        auto get_return_object() {
            print(PRI2, "awaitable2::promise_type::get_return_object()\n");
            return coro_handle::from_promise(*this);
        }

        static awaitable2 get_return_object_on_allocation_failure() {
            print(PRI2, "awaitable2::promise_type::get_return_object_on_allocation_failure()\n");
            throw std::bad_alloc();
        }

        auto initial_suspend() {
            print(PRI2, "awaitable2::promise_type::initial_suspend()\n");
            return std::suspend_never{};
        }

        auto final_suspend() noexcept {
            print(PRI2, "awaitable2::promise_type::final_suspend()\n");
            return std::suspend_never{};
        }

        void unhandled_exception() {
            print(PRI2, "awaitable2::promise_type::unhandled_exception()\n");
            std::terminate();
        }

        void return_value(int v) {
            print(PRI2, "awaitable2::promise_type::return_value(int v)\n");
            m_value = v;
            if (m_awaiting) m_awaiting.resume();
        }

        T m_value;
        std::coroutine_handle<> m_awaiting;
    };

    coro_handle m_coroutine = nullptr;
};

// -----------------------------------------------------------------

awaitable2<int> coroutine2() {
    print(PRI1, "coroutine2(): co_return 42;\n");
    co_return 42;
}

awaitable<int> coroutine1() {
    print(PRI1, "coroutine1(): awaitable2<int> the_coroutine2 = coroutine2();\n");
    awaitable2<int> the_coroutine2 = coroutine2();
    print(PRI1, "coroutine1(): int i = co_await awa2;\n");
    int i = co_await the_coroutine2;
    print(PRI1, "coroutine1(): co_return 42 + i;\n");
    co_return 42 + i;
}

int main() {
    print(PRI1, "main(): awaitable the_coroutine1 = coroutine1();\n");
    awaitable<int> the_coroutine1 = coroutine1();
    print(PRI1, "main(): int i = the_coroutine1.get();\n");
    int i = the_coroutine1.get();
    print(PRI1, "main(): i = %d\n", i);
    return 0;
}
