/**
 * @file p0110_bool.cpp
 * @brief
 * 
 * task::promise_type::final_awaiter::await_suspend() returns bool.
 * task::awaiter::await_suspend() returns void.
 * 
 * Application uses loop_synchronously() and completes_synchronously().
 * 
 * Source: https://godbolt.org/z/7fm8Za
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 * 
 * @author Lewis Baker
 */

///////////////////////////////////////////////////////////////////////////
// Example source code for blog post:
// "C++ Coroutines: Understanding Symmetric-Transfer"
//
// Implementation of a naive 'task' coroutine type using
// bool-returning await_suspend() form.

//#include <experimental/coroutine>
#include <coroutine>
#include <iostream>
#include <atomic>
#include <utility>

//using namespace std::experimental;
using namespace std;

class task {
public:
  class promise_type {
  public:
    task get_return_object() noexcept {
      return task{coroutine_handle<promise_type>::from_promise(*this)};
    }

    suspend_always initial_suspend() noexcept {
      return {};
    }

    void return_void() noexcept {}

    void unhandled_exception() noexcept {
      std::terminate();
    }

    struct final_awaiter {
      bool await_ready() noexcept {
        return false;
      }
      void await_suspend(coroutine_handle<promise_type> h) noexcept {
        auto& promise = h.promise();
        if (promise.ready.exchange(true, std::memory_order_acq_rel)) {
          h.promise().continuation.resume();
        }
      }
      void await_resume() noexcept {}
    };

    final_awaiter final_suspend() noexcept {
      return {};
    }

    coroutine_handle<> continuation;
    std::atomic<bool> ready = false;
  };

  task(task&& t) noexcept
  : coro_(std::exchange(t.coro_, {}))
  {}

  ~task() {
    if (coro_)
      coro_.destroy();
  }

  class awaiter {
  public:
    bool await_ready() noexcept {
      return false;
    }

    bool await_suspend(coroutine_handle<> continuation) noexcept {
      auto& promise = coro_.promise();

      // Store the continuation in the task's promise so that the final_suspend()
      // knows to resume this coroutine when the task completes.
      promise.continuation = continuation;

      // Then we resume the task's coroutine, which is currently suspended
      // at the initial-suspend-point (ie. at the open curly brace).
      coro_.resume();

      // If the final_suspend() method has already set the 'ready'
      // flag to 'true' before we return here then we want to return
      // 'false' from await_supsend() to indicate that the coroutine
      // should be immediately resumed.
      return !promise.ready.exchange(true, std::memory_order_acq_rel);
    }

    void await_resume() noexcept {}
  private:
    friend task;
    explicit awaiter(coroutine_handle<promise_type> h) noexcept
    : coro_(h)
    {}

    coroutine_handle<promise_type> coro_;
  };

  awaiter operator co_await() && noexcept {
    return awaiter{coro_};
  }

private:
  explicit task(coroutine_handle<promise_type> h) noexcept
  : coro_(h)
  {}

  coroutine_handle<promise_type> coro_;
};


struct sync_wait_task {
    struct promise_type {
        sync_wait_task get_return_object() noexcept {
            return sync_wait_task{coroutine_handle<promise_type>::from_promise(*this)};
        }

        suspend_never initial_suspend() noexcept { return{}; }
        
        suspend_always final_suspend() noexcept { return{}; }

        void return_void() noexcept {}

        void unhandled_exception() noexcept { std::terminate(); }
    };

    coroutine_handle<promise_type> coro_;

    explicit sync_wait_task(coroutine_handle<promise_type> h) noexcept : coro_(h) {}

    sync_wait_task(sync_wait_task&& t) noexcept : coro_(t.coro_) {
        t.coro_ = {};
    }

    ~sync_wait_task() {
        if (coro_) {
            coro_.destroy();
        }
    }

    static sync_wait_task start(task&& t) {
        co_await std::move(t);
    }

    bool done() {
        return coro_.done();
    }
};

struct manual_executor {
    struct schedule_op {
        manual_executor& executor_;
        schedule_op* next_ = nullptr;
        coroutine_handle<> continuation_;

        schedule_op(manual_executor& executor)
        : executor_(executor)
        {}

        bool await_ready() noexcept { return false; }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            continuation_ = continuation;
            next_ = executor_.head_;
            executor_.head_ = this;
        }

        void await_resume() noexcept {}
    };

    schedule_op* head_ = nullptr;

    schedule_op schedule() noexcept {
        return schedule_op{*this};
    }

    void drain() {
        while (head_ != nullptr) {
            auto* item = head_;
            head_ = item->next_;
            item->continuation_.resume();
        }
    }

    void sync_wait(task&& t) {
        auto t2 = sync_wait_task::start(std::move(t));
        while (!t2.done()) {
            drain();
        }
    }
};

inline task completes_synchronously() {
    co_return;
}

inline task loop_synchronously(int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously();
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
}

int main() {
    manual_executor ex;
    ex.sync_wait(loop_synchronously(100));
    ex.sync_wait(loop_synchronously(1000));
    ex.sync_wait(loop_synchronously(100'000));
    ex.sync_wait(loop_synchronously(1'000'000));
    return 0;
}
