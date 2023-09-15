/**
 *  Filename: p0400.cpp
 *  Description: Illustrates the use of co_await suspend_always()
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://blog.panicsoftware.com/category/evolution/coroutines/  
 */

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <thread>
#include <string>

#include "print0.h"

//--------------------------------------------------------------

#include <assert.h>
#include <coroutine>

class resumable {

public:
    struct promise_type;

    using coro_handle = std::coroutine_handle<promise_type>;

    resumable(coro_handle handle) : 
        handle_(handle) 
    {
        print("resumable::resumable(coro_handle handle)\n");
        assert(handle); 
    }

    resumable(const resumable& s) = delete;

    resumable(resumable&& s)
        : handle_(s.handle_) {
        print("%p: resumable::resumable(eager&& s)\n", this);
        s.handle_ = nullptr;
    }

    resumable& operator = (const resumable&) = delete;

    resumable& operator = (resumable&& s) {
        print("%p: resumable::resumable = (resumable&& s)\n", this);
        handle_ = s.handle_;
        s.handle_ = nullptr;
        return *this;
    }

    bool resume() {
        print("resumable::resume() - begin\n");
        if (!handle_.done())
            handle_.resume();
        print("resumable::resume() - end\n\n");
        return !handle_.done();
    }

    ~resumable() { 
        print("resumable::~resumable()\n");
        if (handle_) {
            print("resumable::~resumable(): handle_.done() = %d\n", handle_.done());
            if (handle_.done())
                handle_.destroy();
        }
    }

    struct promise_type {

        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type() {
            print("resumable::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print("resumable::promise_type::~promise_type()\n");
        }

        auto get_return_object() {
            print("resumable::promise_type::get_return_object()\n");
            return coro_handle::from_promise(*this);
        }

        auto initial_suspend() { 
            print("resumable::promise_type::initial_suspend()\n");
            return std::suspend_always();
        }
        
        auto final_suspend() noexcept {
            print("resumable::promise_type::final_suspend()\n");
            return std::suspend_always();
        }

        static resumable get_return_object_on_allocation_failure() {
            print("resumable::promise_type::get_return_object_on_allocation_failure()\n");
            throw std::bad_alloc();
        }

        void return_void() {
            print("resumable::promise_type::return_void()\n");
        }

        void unhandled_exception() {
            print("resumable::promise_type::unhandled_exception()\n");
            std::terminate();
        }
    };

private:
    coro_handle handle_;
};

//--------------------------------------------------------------

/*
    // From C:\Program Files\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.23.28105\include\experimental\resumable
    
    // STRUCT suspend_if
    struct suspend_if {
        bool _Ready;

        explicit suspend_if(bool _Condition) noexcept : _Ready(!_Condition) {}

        bool await_ready() noexcept {
            return _Ready;
        }

        void await_suspend(coroutine_handle<>) noexcept {}

        void await_resume() noexcept {}
    };

    // STRUCT suspend_always
    struct suspend_always {
        bool await_ready() noexcept {
            return false;
        }

        void await_suspend(coroutine_handle<>) noexcept {}

        void await_resume() noexcept {}
    };

    // STRUCT suspend_never
    struct suspend_never {
        bool await_ready() noexcept {
            return true;
        }

        void await_suspend(coroutine_handle<>) noexcept {}

        void await_resume() noexcept {}
    };
*/

//--------------------------------------------------------------

struct suspend_always {
    bool await_ready() noexcept {
        print("suspend_always::await_ready()\n");
        return false;
    }

    void await_suspend(std::coroutine_handle<>) noexcept {
        print("suspend_always::await_suspend(...)\n");
    }

    void await_resume() noexcept {
        print("suspend_always::await_resume()\n");
    }
};

//--------------------------------------------------------------

resumable foo() {
    // 00 : resumable::promise_type::promise_type()
    // 00 : resumable::promise_type::initial_suspend()
    // 00 : resumable::promise_type::get_return_object()
    // 00 : resumable::resumable(coro_handle handle)

    print("Hello\n");
    //co_await std::suspend_always();
    co_await std::suspend_always();
    // 00 : suspend_always::await_ready()
    // 00 : suspend_always::await_suspend(...)
    print("Coroutine\n");

    // 00: resumable::promise_type::return_void()
    // 00: resumable::promise_type::final_suspend()
}

//--------------------------------------------------------------

int main() {
    print("main 1\n");
    resumable res = foo();
    print("main 2\n");

    res.resume();
    res.resume();
    res.resume();

    print("main 3\n");

    return 0;
}
