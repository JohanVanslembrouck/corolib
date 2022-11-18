/**
 * @file timer03.cpp
 * @brief
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
{
    print(PRI1, "Timer03::Timer03(...)\n");
}

void Timer03::start()
{
    mainTask();
}

/**
 * @brief Timer03::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer03::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI1, "%p: Timer03::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer03::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);
            
            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI1, "%p: Timer03::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer03::timerTask01a
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
 * @brief Timer03::mainTask
 * @return
 */
async_task<int> Timer03::mainTask()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);
	
	int index1 = get_free_index();
    print(PRI1, "%p: Timer03::mainTask: index1 = %d\n", this, index1);
    async_operation<void> op_timer1{ this, index1 };
    op_timer1.auto_reset(true);

    int index2 = get_free_index();
    print(PRI1, "%p: Timer03::mainTask: index2 = %d\n", this, index2);
    async_operation<void> op_timer2{ this, index2 };
    op_timer2.auto_reset(true);

	async_task<int> t1a = timerTask01a(op_timer1);
    async_task<int> t1b = timerTask01b(op_timer1);
    async_task<int> t1c = timerTask01c(op_timer1);
	
    start_timer_impl(index1, timer1, 1000);

    start_timer_impl(index2, timer2, 1500);
	co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2\n", this);

    start_timer_impl(index1, timer1, 2000);

    start_timer_impl(index2, timer2, 2500);
	co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2\n", this);

    start_timer_impl(index1, timer1, 3000);
    start_timer_impl(index2, timer2, 3500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2\n", this);

    start_timer_impl(index1, timer1, 4000);
    start_timer_impl(index2, timer2, 4500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2\n", this);

    start_timer_impl(index1, timer1, 5000);
    start_timer_impl(index2, timer2, 5500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2\n", this);

	m_running = false;
    co_await t1a;
    print(PRI1, "%p: Timer03::mainTask: after co_await t1a\n", this);
    co_await t1b;
    print(PRI1, "%p: Timer03::mainTask: after co_await t1b\n", this);
    co_await t1c;
    print(PRI1, "%p: Timer03::mainTask: after co_await t1c\n", this);

    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 1;
}
