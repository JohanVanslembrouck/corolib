/**
 * @file timer04.cpp
 * @brief This example shows how one instance of an async_operation<void> object
 * can be used to resume several coroutines that co_wait this object.
 *
 * Coroutines timerTask01a, timerTask01b and timerTask01c are identical (apart from the print lines).
 * In a more elaborate example, these coroutines could perform different actions when they are resumed.
 * 
 * timer04.cpp is a variant of timer03.cpp that only uses a single timer.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer04.h"

#include <corolib/when_all.h>

/**
 * @brief Timer04::Timer04
 * @param ioContext
 */
Timer04::Timer04(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_running(true)
{
    print(PRI1, "Timer04::Timer04(...)\n");
}

/**
 * @brief Timer04::start_timer
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer04::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
    async_operation_base* p_async_op = &async_op;
    
    print(PRI1, "%p: Timer04::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer04::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);
            
            if (!error)
            {
                completionHandler_v_rmc(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer04::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer04::timerTask01a
 * @param op_tmr
 * @return
 */
async_task<int> Timer04::timerTask01a(async_operation_rmc<void>& op_tmr)
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
 * @brief Timer04::timerTask01b
 * @param op_tmr
 * @return
 */
async_task<int> Timer04::timerTask01b(async_operation_rmc<void>& op_tmr)
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
 * @brief Timer04::timerTask01c
 * @param op_tmr
 * @return
 */
async_task<int> Timer04::timerTask01c(async_operation_rmc<void>& op_tmr)
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
 * @brief Timer04::mainTask starts 3 coroutines (timerTask01a, timerTask01b, timerTask01c)
 * that will co_await op_timer1 (that is associated with timer1).
 * mainTask will also co_await op_timer1.
 * mainTask will then start timer1 a number of times.
 * When timer1 expires, the 3 coroutines will be resumed.
 *
 * @return
 */
async_task<int> Timer04::mainTask()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    print(PRI1, "%p: Timer04::mainTask\n", this);
    async_operation_rmc<void> op_timer1{ this };
    op_timer1.auto_reset(true);

    async_task<int> t1a = timerTask01a(op_timer1);
    async_task<int> t1b = timerTask01b(op_timer1);
    async_task<int> t1c = timerTask01c(op_timer1);
    
    start_timer(op_timer1, timer1, 1000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 1000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 1000\n", this);

    start_timer(op_timer1, timer1, 2000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 2000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 2000\n", this);

    start_timer(op_timer1, timer1, 3000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 3000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 3000\n", this);

    start_timer(op_timer1, timer1, 4000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 4000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 4000\n", this);

    start_timer(op_timer1, timer1, 5000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 5000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 5000\n", this);

    // Set m_running to false to make the coroutines leave their loop.
    m_running = false;
    
    // Start the timer: when it expires, the 3 coroutines will leave their loop.
    start_timer(op_timer1, timer1, 1000);
    print(PRI1, "%p: Timer04::mainTask: before co_await op_timer1 1000\n", this);
    co_await op_timer1;
    print(PRI1, "%p: Timer04::mainTask: after co_await op_timer1 1000\n", this);

    when_all wa(t1a, t1b, t1c);
    co_await wa;

    print(PRI1, "--- mainTask: co_return 1;\n");
    co_return 1;
}
