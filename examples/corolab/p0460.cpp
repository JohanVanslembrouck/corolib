/**
 *  Filename: p0460.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *        This example defines a coroutine type that is based
 *        on the coroutine extension of future by Microsoft.
 *        See implementation of await_ready(), await_suspend()
 *        and await_resume().
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/Awaiting
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>

//--------------------------------------------------------------

#include <mutex>
#include <condition_variable>

using namespace std;

class CSemaphore
{
private:
    mutex mutex_;
    condition_variable condition_;
    unsigned int count_;
public:
    CSemaphore() : count_() { }

    void reset() {
        unique_lock<mutex> lock(mutex_);
        count_ = 0;
    }

    void signal() {
        unique_lock<mutex> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    void wait() {
        unique_lock<mutex> lock(mutex_);
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
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t)(*ptr);
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

#include <experimental/resumable>
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

//--------------------------------------------------------------

template<typename T>
struct sync {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    void* address() { return this; }

    T get() {
        print("%p: T sync::get()\n", this);
        if (!coro.done())
            coro.promise().m_sema.wait();
        print("%p: T sync::get(): return coro.promise().m_value;\n", this);
        return coro.promise().m_value;
    }
    
    void wait() {
        print("%p: T sync::wait(): coro.promise().m_sema.wait();\n", this);
        coro.promise().m_sema.wait();
        print("%p: T sync::wait(): : return;\n", this);
    }

    sync(handle_type h)
        : coro(h) {
        print("%p: sync::sync(handle_type h)\n", this);
    }

    bool await_ready() {
        print("%p: sync::await_ready()\n", this);
        const auto ready = coro.done();
        print("%p: sync::await_ready(): return %d\n", this, ready);
        return ready;
    }

    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print("%p: sync::await_suspend(...): entry\n", this);
        this->m_awaitingCoroutine = awaiting;

        std::thread thread1([=]() {
            print("%p: sync::await_suspend(...): thread1: this->wait();\n", this);
            this->wait();
            print("%p: sync::await_suspend(...): thread1: awaiting.resume();\n", this);
            awaiting.resume();
            print("%p: sync::await_suspend(...): thread1: return;\n", this);
            });
        thread1.detach();

        print("%p: sync::await_suspend(...): exit\n", this);
    }

    T await_resume() {
        print("%p: sync::await_resume(): auto r = get()\n", this);
        T r = get();
        print("%p: sync::await_resume(): return r;\n", this);
        return r;
    }
    
    struct promise_type {
        friend struct sync;

        promise_type()
            : m_value(0) {
            print("%p: sync::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: sync::promise::~promise()\n", this);
        }

        auto get_return_object() {
            print("%p: sync::promise_type::get_return_object()\n", this);
            return sync<T>(handle_type::from_promise(*this));
        }

        auto initial_suspend() {
            print("%p: sync::promise_type::initial_suspend()\n", this);
            return std::experimental::suspend_never{};
        }
        
        auto final_suspend() {
            print("%p: sync::promise_type::final_suspend()\n", this);
            return std::experimental::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: sync::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

        void return_value(T v) {
            print("%p: void sync::promise_type::return_value(T V): m_sema.signal()\n", this);
            m_value = v;
            m_sema.signal();
        }

    private:
        CSemaphore m_sema;
        T m_value;
    };
    
    std::experimental::coroutine_handle<> m_awaitingCoroutine;

    handle_type coro;
};

//--------------------------------------------------------------

sync<int> coroutine5() {
    print("coroutine5(): 1\n");
    int i = 42;

    single_consumer_event sync6;
    std::thread thread1([&sync6]() {
        print("coroutine5(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("coroutine5(): thread1: sync6.set();\n");
        sync6.set();
        print("coroutine5(): thread1: return;\n");
        });
    thread1.detach();

    print("coroutine5(): co_await sync6;\n");
    co_await sync6;

    print("coroutine5(): 4: co_return i;\n");
    co_return i;
}

sync<int> coroutine4() {
    print("coroutine4(): 1: sync<int> sync5 = coroutine5();\n");
    sync<int> sync5 = coroutine5();
    print("coroutine4(): 2: int i = co_await sync5;\n");
    int i = co_await sync5;
    print("coroutine4(): 4: co_return i;\n");
    co_return i;
}

sync<int> coroutine3() {
    print("coroutine3(): 1: sync<int> sync4 = coroutine4();\n");
    sync<int> sync4 = coroutine4();
    print("coroutine3(): 2: int i = co_await sync4;\n");
    int i = co_await sync4;
    print("coroutine3(): 4: co_return i;\n");
    co_return i;
}

sync<int> coroutine2() {
    int i;
    for (int j = 0; j < 1; j++)
    {
        print("coroutine2(): 1: sync<int> sync3 = coroutine3();\n");
        sync<int> sync3 = coroutine3();
        print("coroutine2(): 2: int i = co_await sync3;\n");
        i = co_await sync3;
        print("coroutine2(): 4: co_return i;\n");
    }
    co_return i;
}

sync<int> coroutine1() {
    print("coroutine1(): 1: sync<int> sync2 = coroutine2();\n");
    sync<int> sync2 = coroutine2();
    print("coroutine1(): 2: int i = co_await sync2;\n");
    int i = co_await sync2;
    print("coroutine1(): 4: co_return i;\n");
    co_return i;
}

int main() {
    print("main(): 1: sync<int> sync1 = coroutine1();\n");
    sync<int> sync1 = coroutine1();
    print("main(): 2: sync1.address() = %p\n", sync1.address());
    print("main(): 4: int i = sync1.get();\n");
    int i = sync1.get();
    print("main(): 5: i = %d\n", i);
    return 0;
}
