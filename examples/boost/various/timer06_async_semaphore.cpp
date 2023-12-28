/**
 * @file timer06_async_semaphore.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer06_async_semaphore.h"
#include <chrono>

#include <corolib/when_all.h>
#include <corolib/print.h>

Timer06::Timer06(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
    , m_semaphore(4)
{
    print(PRI1, "Timer06::Timer06(...)\n");
}

/**
 * @brief Timer06::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer06::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI2, "%p: Timer06::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer06::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer06::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI2, "%p: Timer06::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI2, "%p: Timer06::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);
            
            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI2, "%p: Timer06::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief subtask1 waits for the semphore, then starts a timer to simulate some asynchronous work
 * and signals the semaphore after the timeout.
 *
 */
async_task<void> Timer06::subTask1(int instance, int timeout)
{
    steady_timer timer1(m_ioContext);

    async_operation<void> op = m_semaphore.wait();
    co_await op;

    print(PRI1, "Timer06::subTask1(%d, %d) got semaphore and will start timer with timeout %d\n", instance, timeout, timeout);

    async_operation<void> op_timer1 = start_timer(timer1, timeout);
    co_await op_timer1;

    print(PRI1, "Timer06::subTask1(%d, %d) going to signal the semaphore\n", instance, timeout);
    m_semaphore.signal();

    co_return;
}

/**
 * @brief subTask2 is a variant of subTask1 that does not use intermediate async_operation<void> variables.
 *
 */
async_task<void> Timer06::subTask2(int instance, int timeout)
{
    steady_timer timer1(m_ioContext);

    co_await m_semaphore.wait();

    print(PRI1, "Timer06::subTask2(%d, %d) got semaphore and will start timer with timeout %d\n", instance, timeout, timeout);

    co_await start_timer(timer1, timeout);

    print(PRI1, "Timer06::subTask2(%d, %d) going to signal the semaphore\n", instance, timeout);
    m_semaphore.signal();

    co_return;
}

async_task<void> Timer06::mainTask()
{
    // The first 4 subTask1 will acquire the semaphore.
    // The other 8 subTask2/subTask1 will be placed in the waiting queue.
    async_task<void> t0 = subTask1(0, 5000);
    async_task<void> t1 = subTask1(1, 6000);
    async_task<void> t2 = subTask1(2, 7000);
    async_task<void> t3 = subTask1(3, 8000);

    async_task<void> t4 = subTask2(4, 5000);
    async_task<void> t5 = subTask2(5, 5000);
    async_task<void> t6 = subTask2(6, 5000);
    async_task<void> t7 = subTask2(7, 5000);

    async_task<void> t8 = subTask1(8, 5000);
    async_task<void> t9 = subTask1(9, 5000);
    async_task<void> t10 = subTask1(10, 5000);
    async_task<void> t11 = subTask1(11, 5000);

    print(PRI1, "mainTask(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all({ &t0, &t1, &t2, &t3, &t4, &t5, &t6, &t7, &t8, &t9, &t10, &t11 });
#else
    co_await when_all(t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11);
#endif
    print(PRI1, "mainTask(): co_return;\n");
    co_return;
}
