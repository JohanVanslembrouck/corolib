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
#if 1

#include <experimental/resumable>

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

#else

#include <experimental/resumable>

template<typename T>
struct sync {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::experimental::coroutine_handle<promise_type>;

    void* address() { return this; }

    T get() {
        print("%p: T sync::get()\n", this);
        if (!coro.promise().set)
            coro.promise().psema->wait();
        print("%p: T sync::get(): coro.promise().psema = %p: return r;\n", this, coro.promise().psema);
        return 42;
    }

    void wait() {
        print("%p: T sync::wait(): coro.promise().psema = %p: coro.promise().psema->wait();\n", this, coro.promise().psema);
        coro.promise().psema->wait();
        print("%p: T sync::wait(): coro.promise().psema = %p: return;\n", this, coro.promise().psema);
    }

    sync(handle_type h)
        : coro(h) {
        print("%p: sync::sync(handle_type h): h.address() = %p, coro.address() = %p\n", this, h.address(), coro.address());
    }

    bool await_ready() {
        print("%p: sync::await_ready()\n", this);
        const auto ready = false;  // coro.done();
        print("%p: sync::await_ready(): ready = %s\n", this, (ready ? "is ready" : "isn't ready"));
        //return coro.done();
        print("%p: sync::await_ready(): return false\n", this);
        return ready;
    }

    void await_suspend(std::experimental::coroutine_handle<> awaiting) {
        print("%p: sync::await_suspend(...): entry\n", this);
        //print("%p: sync::await_suspend(...): awaiting.address() = %p: entry\n", this, awaiting.address());
        m_awaitingCoroutine = awaiting;

        std::thread thread1([=]() {
            print("%p: sync::await_suspend(...): thread1: this->wait();\n", this);
            this->wait();
            print("%p: sync::await_suspend(...): thread1: awaiting.resume();\n", this);
            awaiting.resume();
            print("%p: sync::await_suspend(...): thread1: return;\n", this);
            });
        thread1.detach();

        print("%p: sync::await_suspend(...): exit\n", this);
        //print("%p: sync::await_suspend(...): awaiting.address() = %p: exit\n", this, awaiting.address());
    }

    auto await_resume() {
        //print("%p: sync::await_resume(): coro.address() = %p\n", this, this->coro.address());
        print("%p: sync::await_resume(): auto r = get()\n", this);
        auto r = get();
        //print("Await value is returned: \n");
        print("%p: sync::await_resume(): return r;\n", this);
        return r;
    }

    struct promise_type {
        friend struct sync;

        promise_type() :
            psema(nullptr),
            set(false)
        {
            this->psema = new CSemaphore;
            print("%p: sync::promise_type::promise_type(); psema = %p\n", this, psema);
        }

        ~promise_type() {
            print("%p: sync::promise::~promise()\n", this);
        }

        auto get_return_object() {
            print("%p: sync::promise_type::get_return_object()\n", this);
            //print("%p: sync::promise_type::get_return_object(): this->psema = %p\n", this, this->psema);
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
            //std::exit(1);
        }

        void return_value(T v) {
            print("%p: void sync::promise_type::return_value(T V): psema = %p, psema->signal()\n", this, this->psema);
            psema->signal();
            set = true;
        }

    private:
        CSemaphore* psema;
        bool set;
        T value;
    };

    std::experimental::coroutine_handle<> m_awaitingCoroutine;

    handle_type coro;
};
#endif

//--------------------------------------------------------------

sync<int> coroutine5() {
    print("coroutine5()\n");

    sync<int>::promise_type p;
    sync<int> a = p.get_return_object();
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

sync<int> coroutine4() {
    print("coroutine4(): 1: sync<int> sync5 = coroutine5();\n");
    sync<int> sync5 = coroutine5();
    print("coroutine4(): 2: int i = co_await sync5;\n");
    int i = co_await sync5;
    //print("coroutine4(): 3: sync5.psema() = %p\n", sync5.psema);
    print("coroutine4(): 4: co_return i;\n");
    co_return i;
}

sync<int> coroutine3() {
    print("coroutine3(): 1: sync<int> sync4 = coroutine4();\n");
    sync<int> sync4 = coroutine4();
    print("coroutine3(): 2: int i = co_await sync4;\n");
    int i = co_await sync4;
    //print("coroutine3(): 3: sync4.psema() = %p\n", sync4.psema);
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
        //print("coroutine2(): 3: sync3.psema() = %p\n", sync3.psema);
        print("coroutine2(): 4: co_return i;\n");
    }
    co_return i;
}

sync<int> coroutine1() {
    print("coroutine1(): 1: sync<int> sync2 = coroutine2();\n");
    sync<int> sync2 = coroutine2();
    print("coroutine1(): 2: int i = co_await sync2;\n");
    int i = co_await sync2;
    //print("coroutine1(): 3: sync2.psema() = %p\n", sync2.psema);
    print("coroutine1(): 4: co_return i;\n");
    co_return i;
}

int main() {
    print("main(): 1: sync<int> sync1 = coroutine1();\n");
    sync<int> sync1 = coroutine1();
    print("main(): 2: sync1.address() = %p\n", sync1.address());
    //print("main(): 3: sync1.psema() = %p\n", sync1.psema);
    print("main(): 4: int i = sync1.get();\n");
    int i = sync1.get();
    print("main(): 5: i = %d\n", i);
    return 0;
}

