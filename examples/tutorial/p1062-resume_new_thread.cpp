/** 
 * @file p1060-resume_new_thread.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine5 co_awaits a resume_new_thread object that reverses the conrol flow.
 * See the description of resume_new_thread for more information.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

/**
 * @brief struct resume_new_thread is a variant of resume_same_thread in p1052-resume_same_thread.cpp.
 * await_suspend() launches a thread and detaches it, so await_suspend() will return immediately. 
 * The thread starts a timer of 1000 ms.
 * When the timer expires, the suspended coroutine is resumed from this thread.
 * Resumption of the suspended coroutines is thus on this separate thread.
 *
 */
struct resume_new_thread
{
    bool await_ready() noexcept
    {
        print(PRI1, "resume_new_thread::await_ready()\n");
        return false;
    }

    void await_suspend(std::coroutine_handle<> handle) noexcept {
        print(PRI1, "resume_new_thread::await_suspend(...)\n");
        std::thread(
            [handle] {
                print(PRI1, "std::thread: std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                print(PRI1, "std::thread: before handle.resume();\n");
                handle.resume();
                print(PRI1, "std::thread: after handle.resume();\n\n");
            }
        ).detach();
    }

    void await_resume() noexcept
    {
        print(PRI1, "resume_new_thread::await_resume()\n");
    }
};

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): resume_new_thread\n");
    co_await resume_new_thread();
    int v = 1;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v1 = co_await a1;\n");
    int v1 = co_await a1;

    print(PRI1);
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v2 = co_await a2;\n");
    int v2 = co_await a2;

    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1+v2+1);
    co_return v1+v2+1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

/**
 * @brief Because main() cannot be a coroutine (it cannot return a coroutine type),
 * it cannot use co_await. Instead it calls get_result() on the coroutine object
 * returned from coroutine1().
 */
int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);
        print(PRI1, "main(): async_task<int> a = coroutine1();\n");
        async_task<int> a = coroutine1();
        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
