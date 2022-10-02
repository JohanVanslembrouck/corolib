/**
 *  Filename: p0470.cpp
 *  Description:
 *        Illustrates the use of co_await.
 *
 *        Based upon p0460.cpp, but avoids the introduction
 *        of a dedicated coroutine type to be used by the thread function.
 *        It does, however, some tricky programming using knowledged
 *        of the internals of coroutine types.
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

#include "print0.h"
#include "csemaphore.h"

//--------------------------------------------------------------
#if 1

#include <coroutine>

template<typename T>
struct syncr {

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

    bool await_ready() {
        print("%p: syncr::await_ready()\n", this);
        const auto ready = coro.done();
        print("%p: syncr::await_ready(): return %d\n", this, ready);
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: syncr::await_suspend(...): entry\n", this);
        this->m_awaitingCoroutine = awaiting;

        std::thread thread1([=]() {
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

    struct promise_type {
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

#else

#include <coroutine>

template<typename T>
struct syncr {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    void* address() { return this; }

    T get() {
        print("%p: T syncr::get()\n", this);
        if (!coro.promise().set)
            coro.promise().psema->wait();
        print("%p: T syncr::get(): coro.promise().psema = %p: return r;\n", this, coro.promise().psema);
        return 42;
    }

    void wait() {
        print("%p: T syncr::wait(): coro.promise().psema = %p: coro.promise().psema->wait();\n", this, coro.promise().psema);
        coro.promise().psema->wait();
        print("%p: T syncr::wait(): coro.promise().psema = %p: return;\n", this, coro.promise().psema);
    }

    syncr(handle_type h)
        : coro(h) {
        print("%p: syncr::syncr(handle_type h): h.address() = %p, coro.address() = %p\n", this, h.address(), coro.address());
    }

    bool await_ready() {
        print("%p: syncr::await_ready()\n", this);
        const auto ready = false;  // coro.done();
        print("%p: syncr::await_ready(): ready = %s\n", this, (ready ? "is ready" : "isn't ready"));
        //return coro.done();
        print("%p: syncr::await_ready(): return false\n", this);
        return ready;
    }

    void await_suspend(std::coroutine_handle<> awaiting) {
        print("%p: syncr::await_suspend(...): entry\n", this);
        //print("%p: syncr::await_suspend(...): awaiting.address() = %p: entry\n", this, awaiting.address());
        m_awaitingCoroutine = awaiting;

        std::thread thread1([=]() {
            print("%p: syncr::await_suspend(...): thread1: this->wait();\n", this);
            this->wait();
            print("%p: syncr::await_suspend(...): thread1: awaiting.resume();\n", this);
            awaiting.resume();
            print("%p: syncr::await_suspend(...): thread1: return;\n", this);
            });
        thread1.detach();

        print("%p: syncr::await_suspend(...): exit\n", this);
        //print("%p: syncr::await_suspend(...): awaiting.address() = %p: exit\n", this, awaiting.address());
    }

    auto await_resume() {
        //print("%p: syncr::await_resume(): coro.address() = %p\n", this, this->coro.address());
        print("%p: syncr::await_resume(): auto r = get()\n", this);
        auto r = get();
        //print("Await value is returned: \n");
        print("%p: syncr::await_resume(): return r;\n", this);
        return r;
    }

    struct promise_type {
        friend struct syncr;

        promise_type() :
            psema(nullptr),
            set(false)
        {
            this->psema = new CSemaphore;
            print("%p: syncr::promise_type::promise_type(); psema = %p\n", this, psema);
        }

        ~promise_type() {
            print("%p: syncr::promise::~promise()\n", this);
        }

        auto get_return_object() {
            print("%p: syncr::promise_type::get_return_object()\n", this);
            //print("%p: syncr::promise_type::get_return_object(): this->psema = %p\n", this, this->psema);
            return syncr<T>(handle_type::from_promise(*this));
        }

        auto initial_suspend() {
            print("%p: syncr::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

        auto final_suspend() {
            print("%p: syncr::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print("%p: syncr::promise_type::unhandled_exception()\n", this);
            //std::exit(1);
        }

        void return_value(T v) {
            print("%p: void syncr::promise_type::return_value(T V): psema = %p, psema->signal()\n", this, this->psema);
            psema->signal();
            set = true;
        }

    private:
        CSemaphore* psema;
        bool set;
        T value;
    };

    std::coroutine_handle<> m_awaitingCoroutine;

    handle_type coro;
};
#endif

//--------------------------------------------------------------

syncr<int> coroutine5() {
    print("coroutine5()\n");

    syncr<int>::promise_type p;
    syncr<int> a = p.get_return_object();
    //p.initial_suspend();

    std::thread thread1([&]() {
        print("thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print("thread1: p.set_return_value(42);\n");
        p.return_value(42);
        //p.final_suspend();
        });
    thread1.detach();

    print("coroutine5(): int si = co_await a;\n");
    int v = co_await a;
    print("coroutine5(): co_return %d;\n", v + 1);
    co_return v + 1;
}

syncr<int> coroutine4() {
    print("coroutine4(): 1: syncr<int> syncr5 = coroutine5();\n");
    syncr<int> syncr5 = coroutine5();
    print("coroutine4(): 2: int i = co_await syncr5;\n");
    int i = co_await syncr5;
    //print("coroutine4(): 3: syncr5.psema() = %p\n", syncr5.psema);
    print("coroutine4(): 4: co_return i;\n");
    co_return i;
}

syncr<int> coroutine3() {
    print("coroutine3(): 1: syncr<int> syncr4 = coroutine4();\n");
    syncr<int> syncr4 = coroutine4();
    print("coroutine3(): 2: int i = co_await syncr4;\n");
    int i = co_await syncr4;
    //print("coroutine3(): 3: syncr4.psema() = %p\n", syncr4.psema);
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
        //print("coroutine2(): 3: syncr3.psema() = %p\n", syncr3.psema);
        print("coroutine2(): 4: co_return i;\n");
    }
    co_return i;
}

syncr<int> coroutine1() {
    print("coroutine1(): 1: syncr<int> syncr2 = coroutine2();\n");
    syncr<int> syncr2 = coroutine2();
    print("coroutine1(): 2: int i = co_await syncr2;\n");
    int i = co_await syncr2;
    //print("coroutine1(): 3: syncr2.psema() = %p\n", syncr2.psema);
    print("coroutine1(): 4: co_return i;\n");
    co_return i;
}

int main() {
    print("main(): 1: syncr<int> syncr1 = coroutine1();\n");
    syncr<int> syncr1 = coroutine1();
    print("main(): 2: syncr1.address() = %p\n", syncr1.address());
    //print("main(): 3: syncr1.psema() = %p\n", syncr1.psema);
    print("main(): 4: int i = syncr1.get();\n");
    int i = syncr1.get();
    print("main(): 5: i = %d\n", i);
    return 0;
}

