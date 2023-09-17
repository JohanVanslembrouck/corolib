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

//--------------------------------------------------------------

template<typename T>
struct eager_base : private coroutine_tracker {

    struct promise_type;

    using handle_type = std::coroutine_handle<promise_type>;

    eager_base(const eager_base& s) = delete;

    eager_base(eager_base&& s)
        : coro(s.coro) {
        print("%p: eager_base::eager_base(eager&& s)\n", this);
        s.coro = nullptr;
    }

    ~eager_base() {
        print("%p: eager_base::~eager_base()\n", this);
        if (coro) {
            print("%p: eager_base::~eager_base(): coro.done() = %d\n", this, coro.done());
            if (coro.done())        // Do not destroy if not yet done
                coro.destroy();
        }
    }

    eager_base(handle_type h)
        : coro(h) {
        print("%p: eager_base::eager_base(handle_type h)\n", this);
    }

    eager_base& operator = (const eager_base&) = delete;

    eager_base& operator = (eager_base&& s) {
        print("%p: eager_base::eager_base = (eager_base&& s)\n", this);
        coro = s.coro;
        s.coro = nullptr;
        return *this;
    }

    T get() {
        print("%p: eager_base::get(); coro.done() = %d\n", this, coro.done());
        if (!coro.done()) {
            coro.promise().m_wait_for_signal = true;
            coro.promise().m_sema.wait();
        }
        return coro.promise().m_value;
    }

    struct promise_type : private promise_type_tracker {

        friend struct eager_base;

        promise_type() :
            m_value{},
            m_ready{ false },
            m_awaiting(nullptr),
            m_wait_for_signal(false) {
            print("%p: eager_base::promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print("%p: eager_base::promise_type::~promise_type()\n", this);
        }

        void return_value(T v) {
            print("%p: eager_base::promise_type::return_value(T v): begin\n", this);
            m_value = v;
            m_ready = true;
            if (m_awaiting) {
                print("%p: eager_base::promise_type::return_value(T v): before m_awaiting.resume();\n", this);
                m_awaiting.resume();
                print("%p: eager_base::promise_type::return_value(T v): after m_awaiting.resume();\n\n", this);
            }
            if (m_wait_for_signal) {
                print("%p: eager_base::promise_type::return_value(T v): before m_sema.signal();\n", this);
                m_sema.signal();
                print("%p: eager_base::promise_type::return_value(T v): after m_sema.signal();\n", this);
            }
            print("%p: eager_base::promise_type::return_value(T v): end\n", this);
        }

        struct final_awaiter {
            bool await_ready() const noexcept {
                print("%p: eager_base::promise_type::final_awaiter::await_ready()\n", this);
                return false;
            }

            bool await_suspend(handle_type h) noexcept {
                print("%p: eager_base::promise_type::final_awaiter::await_suspend()\n", this);
                promise_type& promise = h.promise();

                if (promise.m_ready) {
                    print("%p: eager_base::promise_type::final_awaiter::await_suspend(): value ready\n", this);
                    print("%p: eager_base::promise_type::final_awaiter::await_suspend(): m_value = %d\n", this, promise.m_value);
                }
                return !promise.m_ready;
            }

            void await_resume() noexcept {
                print("%p: eager_base::promise_type::final_awaiter::await_resume()\n", this);
            }
        };

        auto final_suspend() noexcept {
            print("%p: eager::promise_type::final_suspend()\n", this);
            return std::suspend_always{};
            //return final_awaiter{};
        }

        auto get_return_object() {
            print("%p: eager_base::promise_type::get_return_object()\n", this);
            return eager_base<T>{handle_type::from_promise(*this)};
        }

        void unhandled_exception() {
            print("%p: eager_base::promise_type::unhandled_exception()\n", this);
            std::exit(1);
        }

    public:
        T m_value;
        bool m_ready;
        CSemaphore m_sema;
        std::coroutine_handle<> m_awaiting;
        bool m_wait_for_signal;
    };

    handle_type coro;
};

//--------------------------------------------------------------

template<typename T>
struct eager : public eager_base<T> {

    struct promise_type;
   
    using handle_type1 = std::coroutine_handle<promise_type>;
    using handle_type = typename eager_base<T>::handle_type;

    eager(handle_type h)
        : eager_base<T>(h) {
        print("%p: eager::eager(handle_type h)\n", this);
    }

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

    struct promise_type : public eager_base<T>::promise_type {

        friend struct eager;

        auto get_return_object()
        {
            print("%p: eager<T>::promise_type::get_return_object()\n");
            return eager<T>{handle_type::from_promise(*this)};
        }

        auto initial_suspend() {
            print("%p: eager::promise_type::initial_suspend()\n", this);
            return std::suspend_never{};
        }

     
    };
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

/*
>------ Build All started: Project: corolib-master, Configuration: x64-Debug ------
  [1/2] Building CXX object examples\corolab\CMakeFiles\p0418exp4.dir\p0418exp4.cpp.obj
  FAILED: examples/corolab/CMakeFiles/p0418exp4.dir/p0418exp4.cpp.obj
  C:\PROGRA~1\MICROS~4\2022\COMMUN~1\VC\Tools\MSVC\1437~1.328\bin\Hostx64\x64\cl.exe  /nologo /TP  -IC:\X\workspace3p\corolib-master\examples\corolab\..\..\include -IC:\local\boost\boost_1_82_0 /await:strict /EHsc /D _WIN32_WINNT=0x0601 /MDd /Zi /Ob0 /Od /RTC1 -std:c++20 /showIncludes /Foexamples\corolab\CMakeFiles\p0418exp4.dir\p0418exp4.cpp.obj /Fdexamples\corolab\CMakeFiles\p0418exp4.dir\ /FS -c C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp
C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(272): error C2664: 
'bool eager_base<T>::promise_type::final_awaiter::await_suspend(std::coroutine_handle<eager_base<T>::promise_type>) noexcept': 
cannot convert argument 1 from 'std::coroutine_handle<eager<int>::promise_type>' to 
                               'std::coroutine_handle<eager_base<T>::promise_type>'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(272): note: 
  No user-defined-conversion operator available that can perform this conversion, or the operator cannot be called
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(122): note: 
  see declaration of 'eager_base<T>::promise_type::final_awaiter::await_suspend'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(272): note: while trying to match the argument list '(std::coroutine_handle<eager<int>::promise_type>)'
C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(281): error C2664: 'bool eager_base<T>::promise_type::final_awaiter::await_suspend(std::coroutine_handle<eager_base<T>::promise_type>) noexcept': cannot convert argument 1 from 'std::coroutine_handle<eager<int>::promise_type>' to 'std::coroutine_handle<eager_base<T>::promise_type>'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(281): note: No user-defined-conversion operator available that can perform this conversion, or the operator cannot be called
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(122): note: see declaration of 'eager_base<T>::promise_type::final_awaiter::await_suspend'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(281): note: while trying to match the argument list '(std::coroutine_handle<eager<int>::promise_type>)'
C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(296): error C2664: 'bool eager_base<T>::promise_type::final_awaiter::await_suspend(std::coroutine_handle<eager_base<T>::promise_type>) noexcept': cannot convert argument 1 from 'std::coroutine_handle<eager<int>::promise_type>' to 'std::coroutine_handle<eager_base<T>::promise_type>'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(296): note: No user-defined-conversion operator available that can perform this conversion, or the operator cannot be called
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(122): note: see declaration of 'eager_base<T>::promise_type::final_awaiter::await_suspend'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(296): note: while trying to match the argument list '(std::coroutine_handle<eager<int>::promise_type>)'
C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(305): error C2664: 'bool eager_base<T>::promise_type::final_awaiter::await_suspend(std::coroutine_handle<eager_base<T>::promise_type>) noexcept': cannot convert argument 1 from 'std::coroutine_handle<eager<int>::promise_type>' to 'std::coroutine_handle<eager_base<T>::promise_type>'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(305): note: No user-defined-conversion operator available that can perform this conversion, or the operator cannot be called
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(122): note: see declaration of 'eager_base<T>::promise_type::final_awaiter::await_suspend'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(305): note: while trying to match the argument list '(std::coroutine_handle<eager<int>::promise_type>)'
C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(314): error C2664: 'bool eager_base<T>::promise_type::final_awaiter::await_suspend(std::coroutine_handle<eager_base<T>::promise_type>) noexcept': cannot convert argument 1 from 'std::coroutine_handle<eager<int>::promise_type>' to 'std::coroutine_handle<eager_base<T>::promise_type>'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(314): note: No user-defined-conversion operator available that can perform this conversion, or the operator cannot be called
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(122): note: see declaration of 'eager_base<T>::promise_type::final_awaiter::await_suspend'
          with
          [
              T=int
          ]
  C:\X\workspace3p\corolib-master\examples\corolab\p0418exp4.cpp(314): note: while trying to match the argument list '(std::coroutine_handle<eager<int>::promise_type>)'
  ninja: build stopped: subcommand failed.


*/

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
