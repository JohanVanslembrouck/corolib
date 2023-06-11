/**
 * @file timer03.cpp
 * @brief This example shows how one instance of an async_operation<void> object
 * can be used to resume several coroutines that co_wait this object.
 *
 * Coroutines timerTask01a, timerTask01b and timerTask01c are identical (apart from the print lines).
 * In a more elaborate example, these coroutines could perform different actions when they are resumed.
 *
 * This requires RESUME_MULTIPLE_COROUTINES = 1 in async_operation.h.
 * Otherwise, only the last coroutine that co_await the object will be resumed.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer03.h"

#include <corolib/when_all.h>

/**
 * @brief Timer03::Timer03
 * @param ioContext
 */
Timer03::Timer03(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_running(true)
{
    print(PRI1, "Timer03::Timer03(...)\n");
}

/**
 * @brief Timer03::start
 */
void Timer03::start()
{
    mainTask();
}

/**
 * @brief Timer03::start_timer
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer03::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
    async_operation_base* p_async_op = &async_op;
    
    print(PRI1, "%p: Timer03::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer03::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);
            
            if (!error)
            {
                completionHandler_v(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer03::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer03::timerTask01a
 * @param op_tmr
 * @return
 */
async_task<int> Timer03::timerTask01a(async_operation<void>& op_tmr)
{
    print(PRI1, "--- timerTask01a: begin\n");
    while (m_running)
    {
        print(PRI1, "--- timerTask01a: before co_await op_tmr\n");
        co_await op_tmr;
        print(PRI1, "--- timerTask01a: after co_await op_tmr\n");
    }
    print(PRI1, "--- timerTask01a: end\n");
    co_return 1;
}

/**
 * @brief Timer03::timerTask01b
 * @param op_tmr
 * @return
 */
async_task<int> Timer03::timerTask01b(async_operation<void>& op_tmr)
{
    print(PRI1, "--- timerTask01b: begin\n");
    while (m_running)
    {
        print(PRI1, "--- timerTask01b: before co_await op_tmr\n");
        co_await op_tmr;
        print(PRI1, "--- timerTask01b: after co_await op_tmr\n");
    }
    print(PRI1, "--- timerTask01b: end\n");
    co_return 1;
}

/**
 * @brief Timer03::timerTask01c
 * @param op_tmr
 * @return
 */
async_task<int> Timer03::timerTask01c(async_operation<void>& op_tmr)
{
    print(PRI1, "--- timerTask01c: begin\n");
    while (m_running)
    {
        print(PRI1, "--- timerTask01c: before co_await op_tmr\n");
        co_await op_tmr;
        print(PRI1, "--- timerTask01c: after co_await op_tmr\n");
    }
    print(PRI1, "--- timerTask01c: end\n");
    co_return 1;
}

/**
 * @brief Timer03::mainTask starts 3 coroutines (timerTask01a, timerTask01b, timerTask01c)
 * that will co_await op_timer1 (that is associated with timer1).
 * mainTask will co_await op_timer2 (that is associated with timer2).
 * mainTask will then start timer1 and timer2 a number of times.
 * When timer1 expires, the 3 coroutines will be resumed.
 *
 * @return
 */
async_task<int> Timer03::mainTask()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    print(PRI1, "%p: Timer03::mainTask\n", this);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);

    print(PRI1, "%p: Timer03::mainTask\n", this);
    async_operation<void> op_timer2{ this };
    op_timer2.auto_reset(true);

    async_task<int> t1a = timerTask01a(op_timer1);
    async_task<int> t1b = timerTask01b(op_timer1);
    async_task<int> t1c = timerTask01c(op_timer1);
    
    start_timer(op_timer1, timer1, 1000);
    start_timer(op_timer2, timer2, 1500);
    print(PRI1, "%p: Timer03::mainTask: before co_await op_timer2 1500\n", this);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 1500\n", this);

    start_timer(op_timer1, timer1, 2000);
    start_timer(op_timer2, timer2, 2500);
    print(PRI1, "%p: Timer03::mainTask: before co_await op_timer2 2500\n", this);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 2500\n", this);

    start_timer(op_timer1, timer1, 3000);
    start_timer(op_timer2, timer2, 3500);
    print(PRI1, "%p: Timer03::mainTask: before co_await op_timer2 3500\n", this);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 3500\n", this);

    start_timer(op_timer1, timer1, 4000);
    start_timer(op_timer2, timer2, 4500);
    print(PRI1, "%p: Timer03::mainTask: before co_await op_timer2 4500\n", this);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 4500\n", this);

    start_timer(op_timer1, timer1, 5000);
    start_timer(op_timer2, timer2, 5500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 5500\n", this);

    // Set m_running to false to make the coroutines leave their loop.
    m_running = false;
    
    // Start the timer: when it expires, the 3 coroutines will leave their loop.
    start_timer(op_timer1, timer1, 1000);
    start_timer(op_timer2, timer2, 1500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 1000\n", this);

    when_all<async_task<int>> wa({ &t1a, &t1b, &t1c });
    co_await wa;

    print(PRI1, "--- mainTask: co_return 1;\n");
    co_return 1;
}
