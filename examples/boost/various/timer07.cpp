/**
 * @file timer07.cpp
 * @brief 
 * See timer07.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "timer07.h"

#include <corolib/when_all.h>

/**
 * @brief Timer07::Timer07
 * @param ioContext
 */
Timer07::Timer07(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_running(true)
{
    print(PRI1, "Timer07::Timer07(...)\n");
}

/**
 * @brief Timer07::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer07::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI1, "%p: Timer07::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer07::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer07::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI1, "%p: Timer07::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer07::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);

            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI1, "%p: Timer07::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer07::start_timer
 * @param async_op
 * @param tmr
 * @param ms
 */
void Timer07::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
    async_operation_base* p_async_op = &async_op;

    print(PRI1, "%p: Timer08::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer08::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);

            if (!error)
            {
                completionHandler_v(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer08::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
                //stop();
            }
        });
}

async_task<int> Timer07::clientTask(int i)
{
    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1 = start_timer(timer1, 1000);

    print(PRI1, "--- Timer07::clientTask(%d): before co_await op_tmr\n", i);
    co_await op_timer1;
    print(PRI1, "--- Timer07::clientTask(%d): after co_await op_tmr\n", i);
    co_return 1;
}

/**
 * @brief Timer07::mainTask
 * This coruutine starts a local timer timer1 and co_awaits its expiration.
 * When the timer expires, mainTask starts coroutine clientTask.
 * The previous actions are repeated NR_TASKS times.
 * Then, mainTask co_awaits the completion of all clientTask instances.
 * It repeats the actions described above 5 times.
 * 
 * @return
 */
async_task<int> Timer07::mainTask()
{
    const int NR_TASKS = 16;

    print(PRI1, "%p: Timer07::mainTask\n", this);

    steady_timer timer1(m_ioContext);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);

    async_task<int> tasks[NR_TASKS];

    // None of the tasks are complete at this point (because none have been started)
    for (int i = 0; i < NR_TASKS; ++i)
    {
        bool ready = tasks[i].is_ready();
        print(PRI1, "%p: Timer07::mainTask: ready = %d\n", this, ready);
    }

    for (int j = 0; j < 5; ++j)
    {
        print(PRI1, "------------------- iteration %d -------------------\n", j);

        for (int i = 0; i < NR_TASKS; ++i)
        {
            start_timer(op_timer1, timer1, 100);
            print(PRI1, "%p: Timer07::mainTask: before co_await op_timer1 100\n", this);
            co_await op_timer1;
            print(PRI1, "%p: Timer07::mainTask: after co_await op_timer1 100\n", this);

            print(PRI1, "%p: Timer07::mainTask: tasks[%d] = std::move(clientTask())\n", this, i);
            tasks[i] = std::move(clientTask(i));
        }

        // 6 tasks are complete at this point
        for (int i = 0; i < NR_TASKS; ++i)
        {
            bool ready = tasks[i].is_ready();
            print(PRI1, "%p: Timer07::mainTask: ready = %d\n", this, ready);
        }

        print(PRI1, "%p: Timer07::mainTask: when_allT<async_task<int>> wa(tasks, NR_TASKS)\n", this);
        when_allT<async_task<int>> wa(tasks, NR_TASKS);

        print(PRI1, "%p: Timer07::mainTask: before co_await wa;\n", this);
        co_await wa;
        print(PRI1, "%p: Timer07::mainTask: after co_await wa;\n", this);

        // All tasks are complete at this point
        for (int i = 0; i < NR_TASKS; ++i)
        {
            bool ready = tasks[i].is_ready();
            print(PRI1, "%p: Timer07::mainTask: ready = %d\n", this, ready);
        }
    }

    print(PRI1, "--- mainTask: co_return 1;\n");
    co_return 1;
}
