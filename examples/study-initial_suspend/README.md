# Study initial_suspend(): lazy or eager?

The examples in this folder are very heavily inspired by the article
https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
and the code in https://godbolt.org/z/-Kw6Nf, https://godbolt.org/z/gy5Q8q, https://godbolt.org/z/7fm8Za 
and https://godbolt.org/z/9baieF

The main purpose of this study is to compare the advantages and disadvantages of
lazy start (initial_suspend() returns std::suspend_always) and
eager start (initial_suspend() returns std::suspend_never) for the three variants of await_suspend(),
and the impact this choice has on the structure of the application code.

await_suspend() can have three return types: void, bool, or coroutine_handle<>.

This is the core example of the article:

```c++
task completes_synchronously() {
  co_return;
}

task loop_synchronously(int count) {
  for (int i = 0; i < count; ++i) {
    co_await completes_synchronously();
  }
}
```

With the împlementation of the task class in https://godbolt.org/z/-Kw6Nf, 
which is repeated here in p1000_void.cpp and in task_void.h,
the application crashes for "large" values of count (depending on your computer).

The reason is that loop_synchronously(int count) 
and completes_synchronously() call each other recursively.

The reason for this mutual recursion is the use of a lazy start coroutine type
(task::promise_type::initial_suspend() returns std::suspend_always) and not so much the fact that 
task::promise_type::final_awaiter::await_suspend(coroutine_handle<promise_type>) and
task::awaiter::await_suspend(coroutine_handle<> ) return void.

By using an eager start coroutine type instead (task::promise_type::initial_suspend() returns std::suspend_never),
as is done in taske_void.h, both coroutines do not call each other recursively anymore.
Instead, the control flow is the same as that of normal functions: the coroutines do not suspend and resume,
but just use call and return.

Another big advantage of using eager start coroutines is that they can be called and started from normal functions,
in particular main().
The reason is that you don't have to co_await such a coroutine to resume them beyound the initial suspend point.
The original code introduces two additinonal classes, manual_executor and sync_wait_task to overcome the "impedance mismatch"
between a normal function (that cannot contain co_await or co_return) and a lway start coroutine.
Notice that sync_wait_task is a coroutine type that uses eager start.

To be completed: a more detailed comparison of the advantages and disadvantage of lazy and eager start coroutines.
