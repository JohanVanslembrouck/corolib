/**
 *  Filename: p0435.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *        Shows the use of a lazy coroutine type, i.e. a coroutine type
 *        that suspends at its beginning and returns control to the calling
 *        function or coroutine.
 *        The calling function or coroutine will resume the called coroutine
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

uint64_t threadids[128];

int get_thread_number(uint64_t id)
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
    //static_assert(sizeof(std::thread::id) == sizeof(uint64_t), 
    //    "this function only works if size of thead::id is equal to the size of uint_64");
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)& id;
    return (*ptr);
}

void print(const char* fmt, ...)
{
    va_list arg;
    va_start(arg, fmt);

    time_t time0;
    time(&time0);
    
    int threadid = get_thread_number(get_thread_id());
    fprintf(stderr, "%02d: ", threadid);

    vfprintf(stderr, fmt, arg);
    va_end(arg);
}

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

            bool await_suspend(std::experimental::coroutine_handle<> awaiter)
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
    std::experimental::coroutine_handle<> m_awaiter;
};

template<typename T>
struct lazy {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    lazy(const lazy& s) = delete;

    lazy(lazy&& s)
        : coro(s.coro) {
        print("lazy::lazy(lazy&& s)\n");
        s.coro = nullptr;
    }

    ~lazy() {
        print("lazy::~lazy()\n");
        //if (coro) coro.destroy();    // can throw exception when lazy goes out of scope. FFS
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

    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
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

    struct promise_type {

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
            return std::experimental::suspend_always{};
        }

        auto final_suspend() {
            print("lazy::promise_type::final_suspend()\n");
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print("lazy::promise::promise_type()\n");
            std::exit(1);
        }

    private:
        T value;
        bool set;
        std::experimental::coroutine_handle<> m_awaiting;
    };

    lazy(handle_type h)
        : coro(h) {
        print("lazy::lazy(handle_type h)\n");
    }

    handle_type coro;
};

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
    int v = 0;
    co_await try5;
#endif
    
    print("coroutine5(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine4() {
    print("coroutine4(): lazy<int> a = coroutine5();\n");
    lazy<int> a = coroutine5();
    print("coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine4(): co_return %d;\n", v + 1);
    co_return v + 1;
}

lazy<int> coroutine3() {
    print("coroutine3(): lazy<int> a = coroutine4();\n");
    lazy<int> a = coroutine4();
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
