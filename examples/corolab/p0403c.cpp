/**
 *  Filename: p0403c.cpp
 *  Description:
 *  Illustrates the use of and the translation of co_await.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *
 */

#include <experimental/coroutine>

using namespace std;

// -----------------------------------------------------------------

/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */

#include <thread>
#include <string>
#include <stdio.h>
#include <stdarg.h>

const int PRI1 = 0x01;
const int PRI2 = 0x02;
const int PRI3 = 0x04;
const int PRI4 = 0x08;

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

const int priority = 0x01;

void print(int pri, const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    if (priority & pri)
        fprintf(stderr, "%02d: %s", threadid, msg);
}

// -----------------------------------------------------------------

struct auto_reset_event {

    std::experimental::coroutine_handle<> m_awaiting;

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
            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
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
    using coro_handle = std::experimental::coroutine_handle<promise_type>;

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
            auto await_suspend(std::experimental::coroutine_handle<> awaiting) noexcept {
                m_awaitable.m_coroutine.resume();
                return awaiting;
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
        using coro_handle = std::experimental::coroutine_handle<promise_type>;

        promise_type() : m_ready(false), m_awaiting(nullptr) { }
        auto get_return_object() { return coro_handle::from_promise(*this); }
        auto initial_suspend() { return std::experimental::suspend_never{}; }
        auto final_suspend() noexcept { return std::experimental::suspend_always{}; }
        void unhandled_exception() { std::terminate(); }
        void return_value(T v) {
            m_value = v;
            m_ready = true;
            if (m_awaiting) m_awaiting.resume();
        }

        T m_value;
        bool m_ready;
        std::experimental::coroutine_handle<> m_awaiting;
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
    print(PRI1, "coroutine1(): i = %d\n", i);
    print(PRI1, "coroutine1(): co_return 42 + i;\n");
    co_return 42 + i;
}

int main() {
    print(PRI1, "main(): awaitable the_coroutine1 = coroutine1();\n");
    awaitable<int> the_coroutine1 = coroutine1();
    //print(PRI1, "main(): are1.resume();\n");
    //are1.resume();
    print(PRI1, "main(): int i = the_coroutine1.get();\n");
    int i = the_coroutine1.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
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
        // int i = co_await coroutine2();
        awaitable<int>&& awaitable1 = coroutine2();
        auto awaiter = awaitable1.operator co_await();
        if (!awaiter.await_ready()) {
            suspend_coroutine
            // await_suspend returns another coroutine_handle
            using coro_handle_t = std::experimental::coroutine_handle<awaitable<int>::promise_type>;
            decltype(awaiter.await_suspend(std::declval<coro_handle_t>())) coro_handle;
            try {
                using handle_t = std::experimental::coroutine_handle<awaitable<int>::promise_type>;
                coro_handle = awaiter.await_suspend(handle_t::from_promise(__context->_promise));
            }
            catch (...) {
                auto exception = std::current_exception();
                goto __resume_point;
            }
            coro_handle.resume();
            return_to_the_caller_or_resumer(__return)
        __resume_point :
            resume_coroutine
        }
        int i = awaiter.await_resume();

        print(PRI1, "coroutine1(): co_return 42 + i;\n");
        //co_return (42 + i);
        __context->_promise.return_value(42 + 1);
        goto __final_suspend;
    }
    catch (...) {
        __context->_promise.unhandled_exception();
    }

__final_suspend:
    co_await __context->_promise.final_suspend();
}
