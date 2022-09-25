/**
 *  Filename: p0403a.cpp
 *  Description:
 *  Illustrates the use of and the translation of co_await.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <coroutine>

using namespace std;

const int priority = 0x01;

#include "print.h"

// -----------------------------------------------------------------

struct auto_reset_event {

    std::coroutine_handle<> m_awaiting;

    auto_reset_event()
        : m_awaiting(nullptr)
        , m_ready(false) {
    }

    auto_reset_event(const auto_reset_event&) = delete;
    auto_reset_event& operator = (const auto_reset_event&) = delete;

    auto_reset_event(auto_reset_event&& s) noexcept
        : m_awaiting(s.m_awaiting)
        , m_ready(s.m_ready) {
        s.m_awaiting = nullptr;
        s.m_ready = false;
    }

    auto_reset_event& operator = (auto_reset_event&& s) noexcept {
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        return *this;
    }

    void resume() {
        m_ready = true;
        if (m_awaiting && !m_awaiting.done())
            m_awaiting.resume();
    }

    auto operator co_await() noexcept
    {
        struct awaiter
        {
            awaiter(auto_reset_event& are_) : m_are(are_) { }

            bool await_ready() {
                return m_are.m_ready;
            }
            void await_suspend(std::coroutine_handle<> awaiting) {
                m_are.m_awaiting = awaiting;
            }
            void await_resume() {
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
struct awaitable
{
    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    awaitable(coro_handle coroutine)
        : m_coroutine(coroutine) {
    }

    ~awaitable() {
        if (m_coroutine)
            m_coroutine.destroy(); 
    }
    
    awaitable() = default;
    awaitable(awaitable const&) = delete;
    awaitable& operator= (awaitable const&) = delete;

    awaitable(awaitable&& other)
        : m_coroutine(other.m_coroutine) {
        other.m_coroutine = nullptr;
    }

    awaitable& operator= (awaitable&& other) {
        if (&other != this) {
            m_coroutine = other.m_coroutine;
            other.m_coroutine = nullptr;
        }
    }

    T get() {
        if (m_coroutine)
            return m_coroutine.promise().m_value;
        return -1;
    }

    // defined in template<typename T> struct awaitable
    auto operator co_await() noexcept {
        struct awaiter {
            awaiter(awaitable& awaitable_) : m_awaitable(awaitable_) { }

            bool await_ready() noexcept {
                return m_awaitable.m_coroutine.promise().m_ready;
            }
            void await_suspend(std::coroutine_handle<> awaiting) noexcept {
                m_awaitable.m_coroutine.promise().m_awaiting = awaiting;
            }
            T await_resume() noexcept {
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

        promise_type() : m_ready(false), m_awaiting(nullptr) { }
        auto get_return_object() { return coro_handle::from_promise(*this); }
        auto initial_suspend() { return std::suspend_never{}; }
        auto final_suspend() noexcept { return std::suspend_always{}; }
        void unhandled_exception() { std::terminate(); }
        void return_value(T v) { 
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
    print(PRI1, "main(): awaitable the_coroutine1 = coroutine1();\n");
    awaitable<int> the_coroutine1 = coroutine1();
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
    print(PRI1, "main(): int i = the_coroutine1.get();\n");
    int i = the_coroutine1.get();
    print(PRI1, "main(): i = %d\n", i);
    return 0;
}

// -----------------------------------------------------------------

void save_frame_pointer(void*) {}
#define suspend_coroutine ;
#define return_to_the_caller_or_resumer(obj) ;
#define resume_coroutine ;

struct __awaitable_int_frame
{
    awaitable<int>::promise_type    _promise;
    unsigned                _i;
    void* _instruction_pointer;
    // storage for registers, etc.
};

awaitable<int> coroutine1_compiled()
{
    __awaitable_int_frame* __context = new __awaitable_int_frame{};
    save_frame_pointer(__context);
    awaitable<int> __return = __context->_promise.get_return_object();
    co_await __context->_promise.initial_suspend();

    try {
        print(PRI1, "coroutine1(): int i = co_await coroutine2();\n");
        int i = co_await coroutine2();
        print(PRI1, "coroutine1(): co_return 42 + i;\n");
        co_return 42 + i;
    }
    catch (...) {
        __context->_promise.unhandled_exception();
    }

__final_suspend:
    co_await __context->_promise.final_suspend();
}

awaitable<int> coroutine1_compiled2()
{
    __awaitable_int_frame* __context = new __awaitable_int_frame{};
    save_frame_pointer(__context);
    awaitable<int> __return = __context->_promise.get_return_object();
    co_await __context->_promise.initial_suspend();

    try {
        print(PRI1, "coroutine1(): int i = co_await coroutine2();\n");
        // int i = co_await coroutine2();
        awaitable<int>&& awaitable1 = coroutine2();
        auto awaiter = awaitable1.operator co_await();
        if (!awaiter.await_ready()) {
            suspend_coroutine
            // await_suspend returns void
            try {
                using handle_t = std::coroutine_handle<awaitable<int>::promise_type>;
                awaiter.await_suspend(handle_t::from_promise(__context->_promise));
                return_to_the_caller_or_resumer(__return)
            }
            catch (...) {
                auto exception = std::current_exception();
                goto __resume_point;
            }
        __resume_point:
            resume_coroutine
        }
        int i = awaiter.await_resume();

        print(PRI1, "coroutine1(): co_return 42 + i;\n");
        //co_return 42 + i;
        __context->_promise.return_value(42 + 1);
        goto __final_suspend;
    }
    catch (...) {
        __context->_promise.unhandled_exception();
    }

__final_suspend:
    co_await __context->_promise.final_suspend();
}


awaitable<int> coroutine1_compiled2a()
{
    __awaitable_int_frame* __context = new __awaitable_int_frame{};
    save_frame_pointer(__context);
    awaitable<int> __return = __context->_promise.get_return_object();
    co_await __context->_promise.initial_suspend();

    try {
        print(PRI1, "coroutine1(): int i = co_await coroutine2();\n");
        // int i = co_await coroutine2();
        awaitable<int>&& awaitable1 = coroutine2();
        auto awaiter = awaitable1.operator co_await();
        if (!awaiter.await_ready()) {
            suspend_coroutine
                // await_suspend returns void
                try {
                using handle_t = std::coroutine_handle<awaitable<int>::promise_type>;
                awaiter.await_suspend(handle_t::from_promise(__context->_promise));
                return_to_the_caller_or_resumer(__return)
            }
            catch (...) {
                auto exception = std::current_exception();
                goto __resume_point;
            }
        __resume_point:
            resume_coroutine
        }
        int i = awaiter.await_resume();

        print(PRI1, "coroutine1(): co_return 42 + i;\n");
        //co_return 42 + i;
        __context->_promise.return_value(42 + 1);
        goto __final_suspend;
    }
    catch (...) {
        __context->_promise.unhandled_exception();
    }

__final_suspend:
    co_await __context->_promise.final_suspend();

    co_return 0;
}

#if 0

int main()
{
    print(PRI1, "main(): coroutine1_compiled2a();\n");
    coroutine1_compiled2a();
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
    print(PRI1, "main(): int i = the_coroutine1.get();\n");
    return 0;
}

#endif
