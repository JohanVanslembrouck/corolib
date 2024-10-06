/**
 * @file timer09.cpp
 * @brief 
 * See timer09.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "timer09.h"

#include <corolib/when_all.h>
#include <corolib/when_any.h>
#include <corolib/taskholder.h>

/**
 * @brief Timer09::Timer09
 * @param ioContext
 */
Timer09::Timer09(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_running(true)
{
    print(PRI1, "Timer09::Timer09(...)\n");
}

/**
 * @brief Timer09::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer09::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI1, "%p: Timer09::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer09::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer09::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI1, "%p: Timer09::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer09::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);

            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI1, "%p: Timer09::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
            }
        });
}

/**
 * @brief Timer09::start_timer
 * @param async_op
 * @param tmr
 * @param ms
 */
void Timer09::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
    async_operation_base* p_async_op = &async_op;
    
    print(PRI1, "%p: Timer09::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer09::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);
            
            if (!error)
            {
                completionHandler_v(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer09::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
            }
        });
}

/**
 * @brief Timer09::clientTask
 * This coroutine starts a timer that simulates the time it takes to serve a client.
 * @param i: identifies a client
 * @return
 */
async_task<int> Timer09::clientTask(int i)
{
    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1 = start_timer(timer1, 1000);

    print(PRI1, "--- Timer09::clientTask(%d): before co_await op_tmr\n", i);
    co_await op_timer1;
    print(PRI1, "--- Timer09::clientTask(%d): after co_await op_tmr\n", i);
    co_return 1;
}

/**
 * @brief Timer09::mainTask
 * This coruutine starts a local timer timer1 and co_awaits its expiration.
 * When the timer expires, mainTask looks for a free entry in taskHolder.
 * If none is free, its execution is suspended and it will be reusmed when at least one is available again.
 * 
 * @return
 */
async_task<int> Timer09::mainTask()
{
    print(PRI1, "%p: Timer09::mainTask\n", this);

    const static int NR_TASKS = 16;
    TaskHolder<NR_TASKS, int> taskHolder;

    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);
 
    for (int j = 0; j < 5 * NR_TASKS; ++j)
    {
        start_timer(op_timer1, timer1, 100);
        print(PRI1, "%p: Timer09::mainTask: before co_await op_timer1;\n", this);
        co_await op_timer1;
        print(PRI1, "%p: Timer09::mainTask: after co_await op_timer1;\n", this);
#if 1
        print(PRI1, "%p: Timer09::mainTask: before co_await taskHolder.wait_free_index();\n", this);
        int i = co_await taskHolder.wait_free_index();
        print(PRI1, "%p: Timer09::mainTask: after co_await taskHolder.wait_free_index();\n", this);
#else
        print(PRI1, "%p: Timer09::mainTask: before co_await taskHolder.wait_free_index_all();\n", this);
        int i = co_await taskHolder.wait_free_index_all();
        print(PRI1, "%p: Timer09::mainTask: after co_await taskHolder.wait_free_index_all();\n", this);
#endif
        // Start a task and mark its entry as in use
        taskHolder.assign(i, std::move(clientTask(j)));
    }

    print(PRI1, "%p: Timer09::mainTask: before co_await taskHolder.wait_all_finished();\n", this);
    co_await taskHolder.wait_all_finished();
    print(PRI1, "%p: Timer09::mainTask: after co_await taskHolder.wait_all_finished();\n", this);

    print(PRI1, "--- mainTask: co_return 1;\n");
    co_return 1;
}
