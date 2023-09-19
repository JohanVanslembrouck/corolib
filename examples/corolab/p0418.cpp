/** 
 *  Filename: p0418.cpp
 *  Description: 
 *        Illustrates the use of co_await.
 *
 *      Shows the use of an eager coroutine tyoe, i.e. a coroutine type
 *      that starts executing and only suspends when it encounters 
 *      a co_await or co_return statement.
 *      The coroutine is resumed from the coroutine or function it called.
 *      To make this possible, 
 *            void await_suspend(std::coroutine_handle<> awaiting)
 *        stores the coroutine handle of the current coroutine in
 *        in the promise of the called coroutine.
 *        When the called coroutine returns by calling co_return, which is
 *        translated by the compiler into return_value(), return_value resumes
 *        the calling coroutine.
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <thread>
#include <coroutine>

#include "print.h"
#include "tracker.h"
#include "csemaphore.h"

#define USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN 1

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
#include <assert.h>
#endif

//--------------------------------------------------------------

template<typename T>
struct eager : private coroutine_tracker {

    struct promise_type;
    friend struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    eager(const eager& s) = delete;

    eager(eager&& s)
        : coro(s.coro) {
        print("%p: eager::eager(eager&& s)\n", this);
        s.coro = nullptr;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        m_promise_type = s.m_promise_type;
        m_promise_valid = s.m_promise_valid;
        s.m_promise_type = nullptr;
        s.m_promise_valid = false;
#endif
    }

    ~eager() {
        print("%p: <<< eager::~eager()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        coroutine_destructor_admin();
#endif
        if (coro) {
            print("%p: <<< eager::~eager(): coro.done() = %d\n", this, coro.done());
            if (coro.done())        // Do not destroy if not yet done
                coro.destroy();
        }
    }

    eager(handle_type h)
        : coro(h) {
        print("%p: >>> eager::eager(handle_type h): promise = %p\n", this, &coro.promise());
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        coro.promise().link_coroutine_object(this);
#endif
    }

    eager& operator = (const eager&) = delete;

    eager& operator = (eager&& s) {
        print("%p: eager::eager = (eager&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        m_promise_type = s.m_promise_type;
        m_promise_valid = s.m_promise_valid;
        s.m_promise_type = nullptr;
        s.m_promise_valid = false;
#endif
        return *this;
    }

    T get() {
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        get_admin();
#endif
        print("%p: eager::get(): coro.done() = %d\n", this, coro.done());
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }
    
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
    void link_promise_type(promise_type* pt) {
        m_promise_type = pt;
        m_promise_valid = true;
    }

    void unlink_promise_type() {
        m_promise_valid = false;
    }

    void coroutine_destructor_admin() {
        print("%p: <<< eager::~eager(): promise = %p (valid = %d)\n", this, m_promise_type, m_promise_valid);
        if (m_promise_valid) {
            m_promise_type->unlink_coroutine_object();
            ++tracker_obj.nr_dying_coroutines_detecting_live_promise;
            assert(&coro.promise() == m_promise_type);
        }
        else
            ++tracker_obj.nr_dying_coroutines_detecting_dead_promise;
    }

    void get_admin() {
        if (!m_promise_valid) {
            print("%p: eager::get(): retrieving value from destructed promise %p!!!\n", this, m_promise_type);
            ++tracker_obj.nr_access_errors;
        }
    }
#endif


    // Alternative 1: define operator co_await and an awaiter type
    // that defines await_ready(), await_suspend() and await_resume().
    
    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(eager& eager_) : 
                m_eager(eager_)
            {}

            bool await_ready() {
                const bool ready = m_eager.coro.done();
                print("%p: eager::await_ready(): return %d;\n", this, ready);
                return ready;
            }

            void await_suspend(std::coroutine_handle<> awaiting) {
                print("%p: eager::await_suspend(std::coroutine_handle<> awaiting)\n", this);
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

    struct promise_type : private promise_type_tracker {

        friend struct eager;

        promise_type() :
            m_value{},
            m_ready{ false },
            m_awaiting(nullptr),
            m_wait_for_signal(false) {
            print("%p: >>> eager::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: <<< eager::promise_type::~promise_type()\n", this);
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            promise_destructor_admin();
#endif
        }

        void return_value(T v) {
            print("%p: eager::promise_type::return_value(T v): begin\n", this);
            m_value = v;
            m_ready = true;
            if (m_awaiting) {
                print("%p: eager::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print("%p: eager::promise_type::return_value(T v): after m_awaiting.resume();\n\n", this);
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
            auto ret = eager<T>{handle_type::from_promise(*this)};
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
            ret.link_promise_type(this);
#endif
            return ret;
        }

        auto initial_suspend() {
            print("%p: eager::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

        struct final_awaiter {
            bool await_ready() const noexcept {
                print("%p: eager::promise_type::final_awaiter::await_ready()\n", this);
                return false;
            }

            bool await_suspend(handle_type h) noexcept {
                print("%p: eager::promise_type::final_awaiter::await_suspend()\n", this);
                promise_type& promise = h.promise();
                
                if (promise.m_ready) {
                    print("%p: eager::promise_type::final_awaiter::await_suspend(): m_ready = %d\n", this, promise.m_ready);
                    print("%p: eager::promise_type::final_awaiter::await_suspend(): m_value = %d\n", this, promise.m_value);
                }
                return !promise.m_ready;
            }

            void await_resume() noexcept {
                print("%p: eager::promise_type::final_awaiter::await_resume()\n", this);
            }
        };

        auto final_suspend() noexcept {
            print("%p: eager::promise_type::final_suspend()\n", this);
            return final_awaiter{};
        }

        void unhandled_exception() {
            print("%p: eager::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        void link_coroutine_object(eager* coroutine_object) {
            m_coroutine_object = coroutine_object;
            m_coroutine_valid = true;
        }

        void unlink_coroutine_object() {
            m_coroutine_valid = false;
        }

        void promise_destructor_admin() {
            print("%p: <<< eager::promise_type::~promise_type(): coroutine_object = %p (valid = %d)\n", this, m_coroutine_object, m_coroutine_valid);
            if (m_coroutine_valid) {
                m_coroutine_object->unlink_promise_type();
                ++tracker_obj.nr_dying_promises_detecting_live_coroutine;
            }
            else {
                ++tracker_obj.nr_dying_promises_detecting_dead_coroutine;
            }
        }
#endif

    private:
        T m_value;
        bool m_ready;
        CSemaphore m_sema;
        std::coroutine_handle<> m_awaiting;
        bool m_wait_for_signal;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
        eager* m_coroutine_object = nullptr;
        bool m_coroutine_valid = false;
#endif
    };

    handle_type coro;
#if USE_COROUTINE_PROMISE_TYPE_LINK_ADMIN
    promise_type* m_promise_type = nullptr;
    bool m_promise_valid = false;
#endif
};

//--------------------------------------------------------------

struct resume_new_thread {
    bool await_ready() noexcept {
        print("resume_new_thread ::await_ready()\n");
        return false;
    }

    void await_suspend(std::coroutine_handle<> handle) noexcept {
        print("resume_new_thread ::await_suspend(...)\n");
        std::thread(
            [handle] {
                print("std::thread: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                print("std::thread: before handle.resume();\n");
                handle.resume();
                print("std::thread: after handle.resume();\n\n");
            }
        ).detach();
        //std::thread(handle).detach();
    }

    void await_resume() noexcept {
        print("resume_new_thread ::await_resume()\n");
    }
};

//--------------------------------------------------------------

eager<int> coroutine5() {
    print("coroutine5(): resume_new_thread\n");
    co_await resume_new_thread();
    int v = 42;
    print("coroutine5(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine4() {
    print("coroutine4(): eager<int> a = coroutine5();\n");
    eager<int> a = coroutine5();
    print("coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine4(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine3() {
    print("coroutine3(): eager<int> a1 = coroutine4();\n");
    eager<int> a1 = coroutine4();
    print("coroutine3(): int v = co_await a1;\n");
    int v1 = co_await a1;

    print(); print("coroutine3(): eager<int> a2 = coroutine4();\n");
    eager<int> a2 = coroutine4();
    print("coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    print("coroutine3(): co_return %d;\n", v1 + v2 + 1);
    co_return v1+v2+1;
}

eager<int> coroutine2() {
    print("coroutine2(): eager<int> a = coroutine3();\n");
    eager<int> a = coroutine3();
    print("coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine2(): co_return %d;\n", v+1);
    co_return v+1;
}

eager<int> coroutine1() {
    print("coroutine1(): eager<int> a = coroutine2();\n");
    eager<int> a = coroutine2();
    print("coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print("coroutine1(): co_return %d;\n", v+1);
    co_return v+1;
}

/**
 * Because main() cannot be a coroutine (it cannot return a coroutine type),
 * it cannot use co_await. Instead it calls get() on the coroutine object
 * returned from coroutine1().
 */
int main() {
    print("main(): eager<int> awa = coroutine1();\n");
    eager<int> awa = coroutine1();
    print("main(): int i = awa.get();\n");
    int i = awa.get();
    print("main(): i = %d\n", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    print("main(): return 0;\n");
    return 0;
}
