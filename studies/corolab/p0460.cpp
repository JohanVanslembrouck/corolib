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
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/Awaiting
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>

#include "print.h"
#include "tracker.h"
#include "csemaphore.h"

//--------------------------------------------------------------

#include <coroutine>
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

            bool await_suspend(std::coroutine_handle<> awaiter)
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
    std::coroutine_handle<> m_awaiter;
};

//--------------------------------------------------------------

template<typename T>
struct syncr : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    void* address() { return this; }

    T get() {
        print("%p: T syncr::get()\n", this);
        if (!coro.done())
            coro.promise().m_sema.wait();
        print("%p: T syncr::get(): return coro.promise().m_value;\n", this);
        return coro.promise().m_value;
    }
    
    void wait() {
        print("%p: T syncr::wait(): coro.promise().m_sema.wait();\n", this);
        coro.promise().m_sema.wait();
        print("%p: T syncr::wait(): : return;\n", this);
    }

    syncr(handle_type h)
        : coro(h) {
        print("%p: syncr::syncr(handle_type h)\n", this);
    }

    ~syncr() {
        print("%p: syncr::~syncr()\n", this);
        if (coro) {
            print("%p: syncr::~syncr(): coro.done() = %d\n", this, coro.done());
            if (coro.done())
                coro.destroy();
        }
    }

    bool await_ready() {
        print("%p: syncr::await_ready()\n", this);
        const auto ready = coro.done();
        print("%p: syncr::await_ready(): return %d\n", this, ready);
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: syncr::await_suspend(...): entry\n", this);
        this->m_awaitingCoroutine = awaiting;

        std::thread thread1([=, this ]() {
            print("%p: syncr::await_suspend(...): thread1: this->wait();\n", this);
            this->wait();
            print("%p: syncr::await_suspend(...): thread1: awaiting.resume();\n", this);
            awaiting.resume();
            print("%p: syncr::await_suspend(...): thread1: return;\n", this);
            });
        thread1.detach();

        print("%p: syncr::await_suspend(...): exit\n", this);
    }

    T await_resume() {
        print("%p: syncr::await_resume(): auto r = get()\n", this);
        T r = get();
        print("%p: syncr::await_resume(): return r;\n", this);
        return r;
    }
    
    struct promise_type : private promise_type_tracker {
        friend struct syncr;

        promise_type()
            : m_value(0) {
            print("%p: syncr::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: syncr::promise::~promise()\n", this);
        }

        auto get_return_object() {
            print("%p: syncr::promise_type::get_return_object()\n", this);
            return syncr<T>(handle_type::from_promise(*this));
        }

        auto initial_suspend() {
            print("%p: syncr::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }
        
        auto final_suspend() noexcept {
            print("%p: syncr::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: syncr::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

        void return_value(T v) {
            print("%p: void syncr::promise_type::return_value(T V): m_sema.signal()\n", this);
            m_value = v;
            m_sema.signal();
        }

    private:
        CSemaphore m_sema;
        T m_value;
    };
    
    std::coroutine_handle<> m_awaitingCoroutine;

    handle_type coro;
};

//--------------------------------------------------------------

syncr<int> coroutine5() {
    print("coroutine5(): 1\n");
    int i = 42;

    single_consumer_event syncr6;
    std::thread thread1([&syncr6]() {
        print("coroutine5(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("coroutine5(): thread1: syncr6.set();\n");
        syncr6.set();
        print("coroutine5(): thread1: return;\n");
        });
    thread1.detach();

    print("coroutine5(): co_await syncr6;\n");
    co_await syncr6;

    print("coroutine5(): 4: co_return i;\n");
    co_return i;
}

syncr<int> coroutine4() {
    print("coroutine4(): 1: syncr<int> syncr5 = coroutine5();\n");
    syncr<int> syncr5 = coroutine5();
    print("coroutine4(): 2: int i = co_await syncr5;\n");
    int i = co_await syncr5;
    print("coroutine4(): 4: co_return i;\n");
    co_return i;
}

syncr<int> coroutine3() {
    print("coroutine3(): 1: syncr<int> syncr4 = coroutine4();\n");
    syncr<int> syncr4 = coroutine4();
    print("coroutine3(): 2: int i = co_await syncr4;\n");
    int i = co_await syncr4;
    print("coroutine3(): 4: co_return i;\n");
    co_return i;
}

syncr<int> coroutine2() {
    int i;
    for (int j = 0; j < 1; j++)
    {
        print("coroutine2(): 1: syncr<int> syncr3 = coroutine3();\n");
        syncr<int> syncr3 = coroutine3();
        print("coroutine2(): 2: int i = co_await syncr3;\n");
        i = co_await syncr3;
        print("coroutine2(): 4: co_return i;\n");
    }
    co_return i;
}

syncr<int> coroutine1() {
    print("coroutine1(): 1: syncr<int> syncr2 = coroutine2();\n");
    syncr<int> syncr2 = coroutine2();
    print("coroutine1(): 2: int i = co_await syncr2;\n");
    int i = co_await syncr2;
    print("coroutine1(): 4: co_return i;\n");
    co_return i;
}

int main() {
    print("main(): 1: syncr<int> syncr1 = coroutine1();\n");
    syncr<int> syncr1 = coroutine1();
    print("main(): 2: syncr1.address() = %p\n", syncr1.address());
    print("main(): 4: int i = syncr1.get();\n");
    int i = syncr1.get();
    print("main(): 5: i = %d\n", i);
    return 0;
}
