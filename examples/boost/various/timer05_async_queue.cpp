/**
 * @file timer05_async_queue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer05_async_queue.h"
#include <chrono>

#include <corolib/when_all.h>
#include <corolib/print.h>

Timer05::Timer05(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
{
    print(PRI1, "Timer05::Timer05(...)\n");
}

void Timer05::start()
{
    mainTasks();
}

/**
 * @brief Timer05::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer05::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI2, "%p: Timer05::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer05::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer05::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI2, "%p: Timer05::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI2, "%p: Timer05::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);
            
            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI2, "%p: Timer05::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief producer() pushes items to the queue, possibly until the queue is full (consumer() not pop-ing).
 * When the queue is full, producer() suspends itself.
 * When producer() notices that consumer() is waiting to pop (queue was empty),
 * it will resume consumer() after it has pushed an item onto the queue.
 */
async_task<void> Timer05::producer(int timeout)
{
    steady_timer timer1(m_ioContext);

    print(PRI1, "producer(): entered\n");
    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        print(PRI1, "producer(): pushing value %d\n", MULTIPLIER * i);
        co_await m_queue.push(MULTIPLIER * i);

        async_operation<void> op_timer1 = start_timer(timer1, timeout);
        co_await op_timer1;
    }
    print(PRI1, "producer(): co_return;\n");
    co_return;
}

/**
 * @brief consumer() removes items from the queue, possibly until the queue is empty (poducer() not pushing).
 * When the queue is empty, consumer() suspends itself.
 * When consumer() notices that producer() is waiting to push (queue was full),
 * it will resume producer() after it has popped an element from the queue.
 */
async_task<void> Timer05::consumer(int timeout)
{
    steady_timer timer1(m_ioContext);

    int val = -1;
    int prev_val = -MULTIPLIER;
    int nrErrors = 0;

    print(PRI1, "consumer(): entered\n");
    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        val = co_await m_queue.pop();
        if (val != (prev_val + MULTIPLIER))
        {
            print(PRI1, "consumer(): prev_val = %d, val = %d\n", prev_val, val);
            nrErrors++;
        }
        prev_val = val;

        async_operation<void> op_timer1 = start_timer(timer1, timeout);
        co_await op_timer1;

        print(PRI1, "consumer(): popped value  %d\n", val);
    }
    print(PRI1, "consumer(): nrErrors = %d\n", nrErrors);
    print(PRI1, "consumer(): co_return;\n");
    co_return;
}

/**
 * @brief Timer05::mainTask1: producer is faster than consumer
 */
async_task<void> Timer05::mainTask1()
{
    print(PRI1, "mainTask1(): async_task<void> p = producer(100);\n");
    async_task<void> p = producer(100);

    print(PRI1, "mainTask1(): async_task<void> c = consumer(200)\n");
    async_task<void> c = consumer(200);

    print(PRI1, "mainTask1(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
	co_await when_all({ &p, &c });
#else
    co_await when_all(p, c);
#endif
    print(PRI1, "mainTask1(); co_return;\n");
    co_return;
}

/**
 * @brief Timer05::mainTask1: consumer is faster than producer
 */
async_task<void> Timer05::mainTask2()
{
    print(PRI1, "mainTask2(): async_task<void> p = producer(200);\n");
    async_task<void> p = producer(200);

    print(PRI1, "mainTask2(): async_task<void> c = consumer(100)\n");
    async_task<void> c = consumer(100);

    print(PRI1, "mainTask2(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all({ &p, &c });
#else
    co_await when_all(p, c);
#endif
    print(PRI1, "mainTask2(); co_return;\n");
    co_return;
}

async_task<void> Timer05::mainTasks()
{
    print(PRI1, "mainTasks(): co_await mainTask1();\n");
    co_await mainTask1();
 
    print(PRI1, "mainTasks(): co_await mainTask2();\n");
    co_await mainTask2();

    print(PRI1, "mainTasks(): co_return;\n");
    co_return;
}
