# Study final_suspend(): avoiding memory leaks

## Introduction

The reader is referred to [awaiter type variants](../../docs/awaiter_type_variants.md) for an introduction to awaiter types.

This text explores final_suspend() and its return types.

See the companion document [Avoiding memory leaks](Avoiding_memory_leaks.md)
for an in-depth discussion of the return types of final_suspend().

This document investigates a memory leak problem that was present in the first implementations of class async_task 
[async_task.h](../../include/corolib/async_task.h) and how it has been solved.
This correct solution(s) emerged from the study applications in this directory.

The following gives an overview of the study applications:

| source file                 | #include X               | #include Y           |
| --------------------------- | ------------------------ | -------------------- |
| p1000e_sa.cpp               | taske_sa.h               | class_sync.h         | 
| p1000_sa.cpp                | task_sa.h                | class_sync.h         |
| p1010e_sa.cpp               | taske_sa.h               | class_async.h        |
| p1010_sa.cpp                | task_sa.h                | class_async.h        |
| p1020e_sa.cpp               | taske_sa.h               | class_async-thread.h |
| p1020_sa.cpp                | task_sa.h                | class_async-thread.h |
|                             |                          |                      |
| p1100e_sn.cpp               | taske_sn.h               | class_sync.h         |
| p1100_sn.cpp                | task_sn.h                | class_sync.h         |
| p1110e_sn.cpp               | taske_sn.h               | class_async.h        |
| p1110_sn.cpp                | task_sn.h                | class_async.h        |
| p1120e_sn.cpp               | taske_sn.h               | class_async-thread.h |
| p1120_sn.cpp                | task_sn.h                | class_async-thread.h |
|                             |                          |                      |
| p1150e_sn2.cpp              | taske_sn2.h              | class_sync.h         |
| p1150_sn2.cpp               | task_sn2.h               | class_sync.h         |
| p1160e_sn2.cpp              | taske_sn2.h              | class_async.h        |
| p1160_sn2.cpp               | task_sn2.h               | class_async.h        |
| p1170e_sn2.cpp              | taske_sn2.h              | class_async-thread.h |
| p1170_sn2.cpp               | task_sn2.h               | class_async-thread.h |
|                             |                          |                      |
| p1200e_void.cpp             | taske_void.h             | class_sync.h         |
| p1200_void.cpp              | task_void.h              | class_sync.h         |
| p1210e_void.cpp             | taske_void.h             | class_async.h        |
| p1210_void.cpp              | task_void.h              | class_async.h        |
| p1220e_void.cpp             | taske_void.h             | class_async-thread.h |
| p1220_void.cpp              | task_void.h              | class_async-thread.h |
|                             |                          |                      |
| p1300e_bool.cpp             | taske_bool.h             | class_sync.h         |
| p1300_bool.cpp              | task_bool.h              | class_sync.h         |
| p1310e_bool.cpp             | taske_bool.h             | class_async.h        |
| p1310_bool.cpp              | task_bool.h              | class_async.h        |
| p1320e_bool.cpp             | taske_bool.h             | class_async-thread.h |
| p1320_bool.cpp              | task_bool.h              | class_async-thread.h |
|                             |                          |                      |
| p1400e_coroutine_handle.cpp | taske_coroutine_handle.h | class_sync.h         |
| p1400_coroutine_handle.cpp  | task_coroutine_handle.h  | class_sync.h         | 
| p1410e_coroutine_handle.cpp | taske_coroutine_handle.h | class_async.h        |
| p1410_coroutine_handle.cpp  | task_coroutine_handle.h  | class_async.h        |
| p1420e_coroutine_handle.cpp | taske_coroutine_handle.h | class_async-thread.h |
| p1420_coroutine_handle.cpp  | task_coroutine_handle.h  | class_async-thread.h |
|                             |                          |                      |
| p1500e_corolib.cpp          | corolib/async_task.h     |                      |
| p1500_corolib.cpp           | corolib/async_task.h     |                      |
| p1510e_corolib.cpp          | corolib/async_task.h     |                      | 
| p1510_corolib.cpp           | corolib/async_task.h     |                      |
| p1520e_corolib.cpp          | corolib/async_task.h     |                      |
| p1520_corolib.cpp           | corolib/async_task.h     |                      |

The 'e' stands for eager start instead of lazy start (default).
See [initial_suspend](../initial_suspend/README.md) for a study of eager and lazy start.

The following table gives an overview of the major differences between the task(e)_xyz.h header files used in this study.

| file                       | final_suspend() returns | await_suspend() return type | resume() call in | return_value() saved to |
| -------------------------- | ----------------------- | --------------------------- | ---------------- | ----------------------- |
| task(e)_sa.h               | std::suspend_always     | void                        | return_value()   | promise_type            |
| task(e)_sn.h               | std::suspend_never      | void                        | return_value()   | promise_type            |
| task(e)_sn2.h              | std::suspend_never      | void                        | return_value()   | task                    |
| task(e)_void.h             | final_awaiter           | void                        | await_suspend()  | promise_type            |
| task(e)_bool.h             | final_awaiter           | bool                        | await_suspend()  | promise_type            |
| task(e)_coroutine_handle.h | final_awaiter           | std::coroutine_handle<>     | infrastructure code | promise_type         |

Files class_sync.h, class_async.h and class_async-thread.h contain a small coroutine application class 
with 4 coroutine member functions.
The difference between the 3 header files is the implementation of the lowest level coroutine4:

* coroutine4 in class_sync.h does not suspend its calling coroutine3: it only contains a co_return statment.
This is called "synchronous completion." All coroutines behave as "ordinary" functions.

* coroutine4 in class_async.h first suspends itself and, consequently, also its calling coroutine3
(which will suspend corutine2, and so on). 
After a short time, coroutine4 will be resumed from main(), and it will then resume coroutine3 (which will resume corutine2,
and so on). This is called "asynchronous completion."

* The implementation of coroutine4 in class_async-thread.h is similar to the one in class_async.h,
except that coroutine4 now resumes coroutine3 on a separate thread.

* From one p1XXX_xyz.cpp source code file, two eexecutables will be produced, stfs-p1XXX_xyz and stfs2-p1XXX_xyz,
where stfs stands for "study final_suspend". If the compiler directive USE_CORO_DONE_TEST=1, stfs-p1XXX_xyz is produced,
if USE_CORO_DONE_TEST=0, stfs2-p1XXX_xyz will be produced:

```c++
#if USE_CORO_DONE_TEST
   ~task() {
        print(PRI2, "%p: task::~task(): test on coro_.done()\n", this);
        if (coro_)
            if (coro_.done()) {
                coro_.destroy();
                coro_ = {};
            }
            else {
                print(PRI2, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    
#else
     ~task() {
        print(PRI2, "%p: task::~task(): no test on coro_.done()\n", this);
        if (coro_) {
            coro_.destroy();
            coro_ = {};
        }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
#endif
```

## final_suspend() returns a standard awaiter type

In this case, final_suspend() returns std::suspend_always or std::suspend_never.

### final_suspend() returns std::suspend_always

Range of applications: p10X0_sa.cpp and p10X0e_sa.cpp. 'sa' stands for suspend_always.

The following is the output of all applications (except from one) built from p10X0_sa.cpp and p10X0e_sa.cpp.

```
00: coroutine1(): co_return 4;
00: main(): int v = a.get_result();
00: main(): v = 4;
00: main(): return 0;
00: 00000008DD39FDC8: task::~task()
00: 0000010B8D1910C0: promise_type::~promise_type()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       1       3       4
00: --------------------------------------------------------
```

Only stfs-p1000e_sa produces a correct output:

```
00: coroutine1(): co_return 4;
00: PRI1, main(): int v = a.get_result();
00: PRI1, main(): v = 4;
00: PRI1, main(): return 0;
00: 0000003A6BAFFCB8: task::~task()
00: 00000207A5FF57F0: promise_type::~promise_type()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       4       0       4
00: --------------------------------------------------------
```

'cons' stands for the number of objects constructed, 'dest' for the number of objects destructed,
'diff' for cons - dest (should be 0!),
'max' for the maximum number of objects alive at any time.

There are two types of objects: 'cor' or coroutine objects (class "task" in this case) and 'pro' or promise_type objects.

The following table gives an overview of all related applications and their results:

| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs-p1000e_sa    | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1000_sa     | correct       | yes           | 4/4 | 4/1 |
| ./stfs-p1010e_sa    | correct       | yes           | 4/4 | 4/1 |
| ./stfs-p1010_sa     | correct       | yes           | 4/4 | 4/1 |
| ./stfs-p1020e_sa    | correct       | yes           | 4/4 | 4/1 |
| ./stfs-p1020_sa     | correct       | yes           | 4/4 | 4/1 |

| program             | get_result()  | memory leaks? | cor | pro | error |
| ------------------- | ------------- | ------------- | --- | --- | ----- |
| ./stfs2-p1000e_sa   | correct       | no            | 4/4 | 4/4 |       |
| ./stfs2-p1000_sa    | correct       | no            | 4/6 | 4/4 |       |
| ./stfs2-p1010e_sa   | correct       | no            | 4/6 | 4/4 |       |
| ./stfs2-p1010_sa    | correct       | no            | 4/6 | 4/4 |       |
| ./stfs2-p1020e_sa   | incorrect     | no            | 4/6 | 4/4 | tcache_thread_shutdown(): unaligned tcache chunk detected |
| ./stfs2-p1020_sa    | ibcorrect     | no            | 4/4 | 4/1 | tcache_thread_shutdown(): unaligned tcache chunk detected |


To obtain the corresponding source file name, omit the prefix stfs- (short for study-final_suspend)
and add .cpp as file name extension.

The result of the test applications is always correct.

Unfortunately, all applications (except from stfs-p1000e_sa) destroy only 3 of the 4 allocated coroutine frames
(with their "embedded" promise_type objects).

This leads to memory leaks because coroutine frames are typically allocated on the heap.
It is easy to overlook these memory leaks and stick to this simple implementation.
This may lead to memory exhaustion related problems in long-running applications.

### final_suspend() returns std::suspend_never

What would happen if final_suspend() returns std::suspend_never instead of std::suspend_always?

The examples p11X0_sn.cpp (lazy start coroutines) and p11X0e_sn.cpp (eager start coroutines) show that the results 
are now *incorrect*, but that all promise_type objects are released. 

X = 0, 1, 2. "sn" stand for suspend_never.

```
00: main(): int v = a.get_result();
00: main(): v = -572662307 = dddddddd;
00: main(): return 0;
00: 000000669F2FF888: task::~task()
00: 000000669F2FF888: task::~task(): !coro.done()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       4       0       4
00: pro 4       4       0       4
00: --------------------------------------------------------
```

There is only 1 application with a correct result, stfs-p1100e_sn (eager start and synchronous completion),
but this application suffers from memory leaks.

The following table gives an overview of all related applications and their results:

| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs-p1100e_sn    | correct       | yes           | 4/1 | 4/1 |
| ./stfs-p1100_sn     | incorrect     | no            | 4/4 | 4/4 |
| ./stfs-p1110e_sn    | incorrect     | no            | 4/4 | 4/4 |
| ./stfs-p1110_sn     | incorrect     | no            | 4/4 | 4/4 |
| ./stfs-p1120e_sn    | incorrect     | no            | 4/4 | 4/4 |
| ./stfs-p1120_sn     | incoorect     | no            | 4/4 | 4/4 |

| program             | get_result()  | memory leaks? | cor | pro | error |
| ------------------- | ------------- | ------------- | --- | --- | ----- |
| ./stfs2-p1100e_sn   | incorrect     | yes           | ?   | ?   | Segmentation fault (core dumped) |
| ./stfs2-p1100_sn    | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1110e_sn   | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1110_sn    | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1120e_sn   | incorrect     | no            | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1120_sn    | incoorect     | no            | ?   | ?   | free(): double free detected in tcache 2 |


The reason is that we read the results from a promise_type object in a coroutine frame that has already been deallocated.
The Windows operating system has filled the memory with 0xdddddddd.
Note that on Ubuntu 22.04 and 24.04, the result is correct, because the memory has not been re-initialized.

### final_suspend() returns std::suspend_never with result stored in the task object

Instead of storing the result in the promise_type object, the previous section seems to indicate that the problem
can be solved by storing the result in the task object, so that we can fetch the result from there.

The examples p11X0_sn.cpp (lazy start coroutines) and p11X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released. X = 5, 6, 7. Subtract 5 to get the corresponding example from the previous section.
Unfortunately, there is again one exception:

The following table gives an overview of all related applications and their results:

| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs-p1150e_sn2   | incorrect     | yes           | 4/1 | 4/1 |
| ./stfs-p1150_sn2    | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1160e_sn2   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1160_sn2    | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1170e_sn2   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1170_sn2    | correct       | no            | 4/4 | 4/4 |

stfs-p1150e_sn2 produces the following result:

```
00: promise_type::~promise_type()
00: main(): int v = a.get_result();
00: main(): v = 0 = 0x0;
00: main(): return 0;
00: 000000D1E7AFFE08: task::~task()
00: 000000D1E7AFFE08: task::~task(): !coro.done()
00: --------------------------------------------------------
00:     cons    dest    diff    max
00: cor 4       1       3       4
00: pro 4       1       3       4
00: --------------------------------------------------------
```

The result is now 0 instead of 4 or a OS-depended "dead beef" value.

The conclusion is that it is not possible to always have
correct results and an absence of memory leaks
using only suspend_always or suspend_never as return value of final_suspend().

| program             | get_result()  | memory leaks? | cor | pro | error |
| ------------------- | ------------- | ------------- | --- | --- | ----- |
| ./stfs2-p1150e_sn2  | incorrect     | yes           | ?   | ?   | Segmentation fault (core dumped) |
| ./stfs2-p1150_sn2   | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1160e_sn2  | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1160_sn2   | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1170e_sn2  | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |
| ./stfs2-p1170_sn2   | incorrect     | yes           | ?   | ?   | free(): double free detected in tcache 2 |


## final_suspend() returns a custom final_awaiter type

The following sections explore the behavior of custom final_awaiter types 
defining their own await_ready(), await_suspend() and await_resume() functions.

The function await_suspend() can have 3 return types: void, bool and coroutine_handle<>.

### await_suspend() returns void

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() return void.

The examples p12X0_sn.cpp (lazy start coroutines) and p12X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released.

The following table gives an overview of all related applications and their results:

| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs-p1200e_void  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1200_void   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1210e_void  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1210_void   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1220e_void  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1220_void   | correct       | no            | 4/4 | 4/4 |


| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs2-p1200e_void | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1200_void  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1210e_void | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1210_void  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1220e_void | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1220_void  | correct       | no            | 4/4 | 4/4 |


### await_suspend() returns bool

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() returns bool.

The examples p13X0_sn.cpp (lazy start coroutines) and p13X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released.

The following table gives an overview of all related applications and their results:

| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs-p1300e_bool  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1300_bool   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1310e_bool  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1310_bool   | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1320e_bool  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1320_bool   | correct       | no            | 4/4 | 4/4 |


| program             | get_result()  | memory leaks? | cor | pro |
| ------------------- | ------------- | ------------- | --- | --- |
| ./stfs2-p1300e_bool | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1300_bool  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1310e_bool | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1310_bool  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1320e_bool | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1320_bool  | correct       | no            | 4/4 | 4/4 |

### await_suspend() returns std::coroutine_handle<>

In this implementation, task::promise_type::final_awaiter::await_suspend() and task::awaiter::await_suspend() return std::coroutine_handle<>.

The examples p14X0_sn.cpp (lazy start coroutines) and p14X0e_sn.cpp (eager start coroutines) show that the results are correct and
that all promise_type objects are released.

The following table gives an overview of all related applications and their results:

| program                        | get_result()  | memory leaks? | cor | pro |
| ------------------------------ | ------------- | ------------- | --- | --- |
| ./stfs-p1400e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1400_coroutine_handle  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1410e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1410_coroutine_handle  | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1420e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs-p1420_coroutine_handle  | correct       | no            | 4/4 | 4/4 |

| program                         | get_result()  | memory leaks? | cor | pro |
| ------------------------------- | ------------- | ------------- | --- | --- |
| ./stfs2-p1400e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1400_coroutine_handle  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1410e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1410_coroutine_handle  | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1420e_coroutine_handle | correct       | no            | 4/4 | 4/4 |
| ./stfs2-p1420_coroutine_handle  | correct       | no            | 4/4 | 4/4 |


## Using corolib async_task and async_ltask

The examples p15X0_sn.cpp (lazy start coroutines using async_ltask) and p15X0e_sn.cpp (eager start coroutines using async_task)
show that the results are correct and that all promise_type objects are released.

The following table gives an overview of all related applications and their results:

| program                | get_result()  | memory leaks? | cor | pro | fin |
| ---------------------- | ------------- | ------------- | --- | --- | --- |
| ./stfs-p1500e_corolib  | correct       | no            | 4/4 | 4/4 | 4/4 |
| ./stfs-p1500_corolib   | correct       | no            | 4/4 | 4/4 | 4/4 |
| ./stfs-p1510e_corolib  | correct       | no            | 4/4 | 4/4 | 4/4 |
| ./stfs-p1510_corolib   | correct       | no            | 4/4 | 4/4 | 4/4 |
| ./stfs-p1520e_corolib  | correct       | no            | 4/4 | 4/4 | 4/4 |
| ./stfs-p1520_corolib   | correct       | no            | 4/4 | 4/4 | 4/4 |

## Conclusion

Only using a custom final_awaiter type is it possible to develop applications that always produce a correct result
and release all coroutine frames.
