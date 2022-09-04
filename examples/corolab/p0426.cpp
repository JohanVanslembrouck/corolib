/**
 *  Filename: p0426.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *        Variant of p0420.cpp. See p0420.cpp for more details.
 *
 *        Uses a dedicated coroutine type (auto_reset_event).
 *
 *        coroutine5 starts coroutine6 twice. Both instances proceed independently 
 *        if "someone" resumes the auto_reset_event passed as their first argument.
 *        Each coroutine6 instance has to be resumed twice before it will resume 
 *        its second argument (passed by and co_awaited by coroutine5).
 *        The main() function is responsible for resuming both instances of coroutine6 twice.
 *        The order in which this happens is not important.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>

#include <experimental/resumable>

#include <mutex>
#include <condition_variable>

//--------------------------------------------------------------

class CSemaphore
{
private:
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned int count_;
public:
    CSemaphore() : count_() { }

    void reset() {
        std::unique_lock<std::mutex> lock(mutex_);
        count_ = 0;
    }

    void signal() {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait() {
        std::unique_lock < std::mutex > lock(mutex_);
        while (!count_)
            condition_.wait(lock);
        --count_;
    }
};

//--------------------------------------------------------------

/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */

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
    uint64_t* ptr = (uint64_t*)& id;
    return (*ptr);
}

void print()
{
    fprintf(stderr, "\n");
}

void print(const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
        get_thread_number32((uint32_t)get_thread_id()) :
        get_thread_number64(get_thread_id());
    fprintf(stderr, "%02d: %s", threadid, msg);
}

//--------------------------------------------------------------

struct auto_reset_event {

    std::experimental::coroutine_handle<> m_awaiting;

    auto_reset_event()
        : m_awaiting(nullptr)
        , m_ready(false) {
        print("%p: auto_reset_event::auto_reset_event()\n", this);
    }

    auto_reset_event(const auto_reset_event&) = delete;
    auto_reset_event& operator = (const auto_reset_event&) = delete;

    auto_reset_event(auto_reset_event&& s)
        : m_awaiting(s.m_awaiting) 
        , m_ready(s.m_ready) {
        print("%p: auto_reset_event::auto_reset_event(auto_reset_event&& s)\n", this);
        s.m_awaiting = nullptr;
        s.m_ready = false;
    }

    auto_reset_event& operator = (auto_reset_event&& s) {
        print("%p: auto_reset_event::auto_reset_event = (auto_reset_event&& s)\n", this);
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        return *this;
    }

    void resume() {
        print("%p: auto_reset_event::resume(): before m_awaiting.resume();\n", this);
        m_ready = true;
        if (m_awaiting && !m_awaiting.done())
            m_awaiting.resume();
        print("%p: auto_reset_event::resume(): after m_awaiting.resume();\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:
            awaiter(auto_reset_event& are_) :
                m_are(are_) {
                print("%p: auto_reset_event::awaiter(auto_reset_event& ars_)\n", this);
            }

            bool await_ready() {
                print("%p: auto_reset_event::await_ready(): return false\n", this);
                return m_are.m_ready;
            }

            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
                print("%p: auto_reset_event::await_suspend(std::experimental::coroutine_handle<> awaiting)\n", this);
                m_are.m_awaiting = awaiting;
            }

            void await_resume() {
                print("%p: void auto_reset_event::await_resume()\n", this);
                m_are.m_ready = false;
            }

        private:
            auto_reset_event& m_are;
        };

        return awaiter{ *this };
    }

    bool m_ready;
};

//--------------------------------------------------------------

template<typename T>
struct eager {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

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
        print("%p: eager::get()\n");
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:
            awaiter(eager& eager_) : 
                m_eager(eager_) {
                print("%p: awaiter::awaiter(eager& eager_)\n", this);
            }

            bool await_ready() {
                bool ready = m_eager.coro.done();
                print("%p: eager::await_ready(): return %d\n", this, ready);
                return ready;
            }

            void await_suspend(std::experimental::coroutine_handle<> awaiting) {
                print("%p: eager::await_suspend(std::experimental::coroutine_handle<> awaiting)\n", this);
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

    struct promise_type  {

        friend struct eager;

        promise_type() :
            m_value(0),
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
            return std::experimental::suspend_never{};
        }

        auto final_suspend() noexcept {
            print("%p: eager::promise_type::final_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: eager::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        CSemaphore m_sema;
        bool m_wait_for_signal;
        std::experimental::coroutine_handle<> m_awaiting;
    };

    handle_type coro;
};

//--------------------------------------------------------------

struct oneway_task
{
    struct promise_type
    {
        std::experimental::suspend_never initial_suspend() {
            print("%p: oneway_task::promise_type::initial_suspend()\n", this);
            return {};
        }

        std::experimental::suspend_never final_suspend() noexcept {
            print("%p: oneway_task::promise_type::final_suspend()\n", this);
            return {};
        }

        void unhandled_exception() {
            print("%p: oneway_task::promise_type::unhandled_exception()\n", this);
            std::terminate();
        }

        oneway_task get_return_object() {
            print("%p: oneway_task::promise_type::get_return_object()\n", this);
            return {};
        }

        void return_void() {
            print("%p: oneway_task::promise_type::return_void()\n", this);
        }
    };
};

//--------------------------------------------------------------
//--------------------------------------------------------------

auto_reset_event m1, m1a, m2, m2a;

oneway_task coroutine6(const char* name, auto_reset_event &m, auto_reset_event& ma)
{
    print("coroutine6(%s, ...): co_await m;\n", name);
    co_await m;
    print("coroutine6(%s, ...): co_await m;\n", name);
    co_await m;
    print("coroutine6(%s, ...): ma.resume();\n", name);
    ma.resume();
    print("coroutine6(%s, ...): return;\n", name);
}

eager<int> coroutine5() {
    print("coroutine5(): coroutine6(m1, m1, m1a);\n");
    (void) coroutine6("m1", m1, m1a);
    print("coroutine5(): coroutine6(m2, m2, m2a);\n");
    (void) coroutine6("m2", m2, m2a);

    print("coroutine5(): co_await m1a;\n");
    co_await m1a;
    print("coroutine5(): co_await m2a;\n");
    co_await m2a;

    print("coroutine5(): int v = 42;\n");
    int v = 42;
    print("coroutine5(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine4() {
    print("coroutine4(): eager<int> a5 = coroutine5();\n");
    eager<int> a5 = coroutine5();
    print("coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print("coroutine4(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine3() {
    print("coroutine3(): eager<int> a4 = coroutine4();\n");
    eager<int> a4 = coroutine4();
    print("coroutine3(): int v = co_await a4;\n");
    int v1 = co_await a4;
    print("coroutine3(): co_return %d;\n", v1 + 1);
    co_return v1+1;
}

eager<int> coroutine2() {
    print("coroutine2(): eager<int> a3 = coroutine3();\n");
    eager<int> a3 = coroutine3();
    print("coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print("coroutine2(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine1() {
    print("coroutine1(): eager<int> a2 = coroutine2();\n");
    eager<int> a2 = coroutine2();
    print("coroutine1(): int v = co_await a2;\n");
    int v = co_await a2;
    print("coroutine1(): co_return %d;\n", v+1);
    co_return v+1;
}

int main() {
    print("main(): eager<int> a1 = coroutine1();\n");
    eager<int> a1 = coroutine1();

    print(); print("main(): m2.resume();\n");
    m2.resume();
    print(); print("main(): m1.resume();\n");
    m1.resume();
    print(); print("main(): m1.resume();\n");
    m1.resume();
    print(); print("main(): m2.resume();\n");
    m2.resume();

    print(); print("main(): int i = a1.get();\n");
    int i = a1.get();
    print("main(): i = %d\n", i);
    return 0;
}
