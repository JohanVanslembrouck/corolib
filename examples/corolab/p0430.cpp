/** 
 *  Filename: p0430.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *      Shows the use of a lazy coroutine type, i.e. a coroutine type
 *      that suspends at its beginning and returns control to the calling
 *        function or coroutine.
 *      The calling function or coroutine will resume the called coroutine
 *        from its get() function.
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

// -------------------------------------------------------------

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
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t) (*ptr);
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

// -------------------------------------------------------------

#include <mutex>
#include <condition_variable>

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

// -------------------------------------------------------------

template<typename T>
struct lazy {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    lazy(const lazy& s) = delete;

    lazy(lazy&& s)
        : coro(s.coro) {
        print("%p: lazy::lazy(lazy&& s)\n", this);
        s.coro = nullptr;
    }

    ~lazy() {
        print("%p: lazy::~lazy()\n", this);
        if (coro)
            coro.destroy();
    }

    lazy& operator = (const lazy&) = delete;

    lazy& operator = (lazy&& s) {
        print("%p: lazy::lazy = (lazy&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("%p: lazy::get(): coro.resume();\n", this);
        coro.resume();

        print("%p: lazy::get(): spinlock\n", this);
        while (!coro.done())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        print("%p: lazy::get(): coro.promise().m_value;\n", this);
        return coro.promise().m_value;
    }

    bool await_ready() {
        const auto ready = coro.done();
        print("%p: lazy::await_ready(): return %d;\n", this, ready);
        return ready;
    }

#if 0
    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print("%p: lazy::await_suspend(...): coro.resume();\n", this);
        coro.resume();
        print("%p: lazy::await_suspend(...): awaiting.resume();\n", this);
        awaiting.resume();
        print("%p: lazy::await_suspend(...): return;\n", this);
    }
#else
    std::experimental::coroutine_handle<> await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print("%p: lazy::await_suspend(...): coro.resume();\n", this);
        coro.resume();
        print("%p: lazy::await_suspend(...): return awaiting;\n", this);
        return awaiting;
    }
#endif

    auto await_resume() {
        print("%p: lazy::await_resume()\n", this);
        const auto r = coro.promise().m_value;
        print("%p: lazy::await_resume(): coro.done() = %d\n", this, coro.done());
        return r;
    }

    struct promise_type {

        friend struct lazy;

        promise_type() : m_value(0) {
            print("%p: lazy::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: lazy::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print("%p: lazy::promise_type::return_value(T v)\n", this);
            this->m_value = v;
        }

        auto get_return_object() {
            print("%p: lazy::promise_type::get_return_object()\n", this);
            return lazy<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("%p: lazy::promise_type::initial_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        auto final_suspend() {
            print("%p: lazy::promise_type::final_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: lazy::promise::promise_type()\n", this);
            std::exit(1);
        }

    private:
        T m_value;
        std::experimental::coroutine_handle<> m_awaiting;
    };

    lazy(handle_type h)
        : coro(h) {
        print("%p: lazy::lazy(handle_type h)\n", this);
    }

    handle_type coro;
};

// -------------------------------------------------------------

lazy<int> coroutine5() {
    print("coroutine5()\n");

#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    
    CSemaphore sema;

    std::thread thread1([&]() {
        print("thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("thread1: sema.signal();\n");
        sema.signal();
        });
    thread1.detach();

    print("coroutine5(): sema.wait();\n");
    sema.wait();
#endif

    int v = 42;

    print("coroutine5(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> function4() {
    print("function4(): lazy<int> a = coroutine5();\n");
    lazy<int> a = coroutine5();
    print("function4(): int v = co_await a;\n");
    int v = co_await a;
    print("function4(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine3() {
    print("coroutine3(): lazy<int> a = function4();\n");
    lazy<int> a = function4();
    print("coroutine3(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine3(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine2() {
    print("coroutine2(): lazy<int> a = coroutine3();\n");
    lazy<int> a = coroutine3();
    print("coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine2(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine1() {
    print("coroutine1(): lazy<int> a = coroutine2();\n");
    lazy<int> a = coroutine2();
    print("coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine1(): co_return %d;\n", v + 1);
    co_return v + 1;
}

int main() {
    print("main(): lazy<int> awa = coroutine1();\n");
    lazy<int> awa = coroutine1();
    print("main(): int i = awa.get();\n");
    int i = awa.get();
    print("main(): i = %d\n", i);
    print("main(): return 0;\n");
    return 0;
}
