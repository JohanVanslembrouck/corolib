/**
 * @file timer01.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#include "timer01.h"

#include <corolib/wait_all_awaitable.h>

Timer01::Timer01(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
{
    print(PRI1, "Timer01::Timer01(...)\n");
}

void Timer01::start()
{
    mainTask();
}

async_operation<void> Timer01::start_timer(steady_timer& timer, int ms)
{
	index = (index + 1) & (NROPERATIONS - 1);
	print(PRI1, "%p: CommCore::start_timer(timer, %d): index = %d\n", this, ms, index);
	assert(m_async_operations[index] == nullptr);
	async_operation<void> ret{ this, index };
	start_tmr(index, timer, ms);
	return ret;
}

void Timer01::start_tmr(const int idx, steady_timer& tmr, int ms)
{
	print(PRI1, "%p: Timer01::start_tmr()\n", this);

	tmr.expires_after(std::chrono::milliseconds(ms));

	tmr.async_wait(
		[this, idx](const boost::system::error_code& error)
		{
			print(PRI1, "%p: Timer01::handle_timer(): idx = %d, entry\n", this, idx);
			async_operation_base* om_async_operation = m_async_operations[idx];

			if (!error)
			{
				print(PRI1, "%p: Timer01::handle_timer(): idx = %d, om_async_operation = %p\n", this, idx, om_async_operation);
				if (om_async_operation)
				{
					om_async_operation->completed();
				}
				else
				{
					// This can occur when the async_operation_base has gone out of scope.
					print(PRI1, "%p: Timer01::handle_timer(): idx = %d, Warning: om_async_operation == nullptr\n", this, idx);
				}
			}
			else
			{
				print(PRI1, "%p: Timer01::handle_timer(): idx = %d, Error on timer: %s\n", this, idx, error.message().c_str());
				//stop();
			}
		});
}


/**
 * @brief Timer01::timerTask01
 * @return
 */
async_task<int> Timer01::timerTask01()
{
    steady_timer timer1(m_ioContext);

    async_operation<void> op_timer1 = start_timer(timer1, 0);
    op_timer1.auto_reset(true);
    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 0\n");

    for (int i = 0; i < 3; i++)
    {
        async_operation<void> op_timer1a = start_timer(timer1, 1000);
        co_await op_timer1a;
        print(PRI1, "--- timerTask01: after co_await op_timer1a --- 1000\n");

        async_operation<void> op_timer1b = start_timer(timer1, 2000);
        co_await op_timer1b;
        print(PRI1, "--- timerTask01: after co_await op_timer1b --- 2000\n");

        async_operation<void> op_timer1c = start_timer(timer1, 0);
        co_await op_timer1c;
        print(PRI1, "--- timerTask01: after co_await op_timer1c --- 3000\n");

        async_operation<void> op_timer1d = start_timer(timer1, 4000);
        co_await op_timer1d;
        print(PRI2, "--- timerTask01: after co_await op_timer1d --- 4000\n");

        async_operation<void> op_timer1e = start_timer(timer1, 5000);
        co_await op_timer1e;
        print(PRI1, "--- timerTask01: after co_await op_timer1e --- 5000\n");
    }

    co_return 1;
}

/**
 * @brief Timer01::timerTask02
 * @return
 */
async_task<int> Timer01::timerTask02()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    async_operation<void> op_timer1 = start_timer(timer1, 0);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 0);
    op_timer2.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask02: after co_await op_timer1 --- 0\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask02: after co_await op_timer2 --- 0\n");

    for (int i = 0; i < 3; i++)
    {
        async_operation<void> op_timer1a = start_timer(timer1, 1000);
        async_operation<void> op_timer2a = start_timer(timer2, 1500);

        co_await op_timer1a;
        print(PRI1, "--- timerTask02: after co_await op_timer1a --- 1000\n");
        co_await op_timer2a;
        print(PRI1, "--- timerTask02: after co_await op_timer2b --- 1500\n");

        async_operation<void> op_timer1b = start_timer(timer1, 2000);
        async_operation<void> op_timer2b = start_timer(timer2, 2500);

        co_await op_timer1b;
        print(PRI1, "--- timerTask02: after co_await op_timer1b --- 2000\n");
        co_await op_timer2b;
        print(PRI1, "--- timerTask02: after co_await op_timer2b --- 2500\n");

        async_operation<void> op_timer1c = start_timer(timer1, 3000);
        async_operation<void> op_timer2c = start_timer(timer2, 3500);

        co_await op_timer1c;
        print(PRI1, "--- timerTask02: after co_await op_timer1c --- 3000\n");
        co_await op_timer2c;
        print(PRI1, "--- timerTask02: after co_await op_timer2c --- 3500\n");

        async_operation<void> op_timer1d = start_timer(timer1, 4000);
        async_operation<void> op_timer2d  = start_timer(timer2, 4500);

        co_await op_timer1d;
        print(PRI1, "--- timerTask02: after co_await op_timer1d --- 4000\n");
        co_await op_timer2d;
        print(PRI1, "--- timerTask02: after co_await op_timer2d --- 4500\n");

        async_operation<void> op_timer1e = start_timer(timer1, 5000);
        async_operation<void> op_timer2e = start_timer(timer2, 5500);

        co_await op_timer1e;
        print(PRI1, "--- timerTask02: after co_await op_timer1e --- 5000\n");
        co_await op_timer2e;
        print(PRI1, "--- timerTask02: after co_await op_timer2e --- 5500\n");
    }

    co_return 1;
}

/**
 * @brief Timer01::timerTask03
 * @return
 */
async_task<int> Timer01::timerTask03()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    async_operation<void> op_timer1 = start_timer(timer1, 0);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 0);
    op_timer2.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask03: after co_await op_timer1 --- 0\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask03: after co_await op_timer2 --- 0\n");

    for (int i = 0; i < 3; i++)
    {
        async_operation<void> op_timer1a = start_timer(timer1, 1000);
        async_operation<void> op_timer2a = start_timer(timer2, 1500);

        co_await op_timer2a;
        print(PRI1, "--- timerTask03: after co_await op_timer2a --- 1500\n");
        co_await op_timer1a;
        print(PRI1, "--- timerTask03: after co_await op_timer1a --- 1000\n");

        async_operation<void> op_timer1b = start_timer(timer1, 2000);
        async_operation<void> op_timer2b = start_timer(timer2, 2500);

        co_await op_timer2b;
        print(PRI1, "--- timerTask03: after co_await op_timer2b --- 2500\n");
        co_await op_timer1b;
        print(PRI1, "--- timerTask03: after co_await op_timer1b --- 2000\n");
        
        async_operation<void> op_timer1c = start_timer(timer1, 3000);
        async_operation<void> op_timer2c = start_timer(timer2, 3500);

        co_await op_timer2c;
        print(PRI1, "--- timerTask03: after co_await op_timer2c --- 3500\n");
        co_await op_timer1c;
        print(PRI1, "--- timerTask03: after co_await op_timer1c --- 3000\n");
       
        async_operation<void> op_timer1d = start_timer(timer1, 4000);
        async_operation<void> op_timer2d = start_timer(timer2, 4500);

        co_await op_timer2d;
        print(PRI1, "--- timerTask03: after co_await op_timer2d --- 4500\n");
        co_await op_timer1d;
        print(PRI1, "--- timerTask03: after co_await op_timer1d --- 4000\n");
       
        async_operation<void> op_timer1e = start_timer(timer1, 5000);
        async_operation<void> op_timer2e = start_timer(timer2, 5500);

        co_await op_timer2e;
        print(PRI1, "--- timerTask03: after co_await op_timer2e --- 5500\n");
        co_await op_timer1e;
        print(PRI1, "--- timerTask03: after co_await op_timer1e --- 5000\n");
    }

    co_return 1;
}

/**
 * @brief Timer01::mainTask
 * @return
 */
async_task<int> Timer01::mainTask()
{

    async_task<int> t1 = timerTask01();
    async_task<int> t2 = timerTask02();
    async_task<int> t3 = timerTask03();

    wait_all<async_task<int>> wa({ &t1, &t2, &t3 });
    co_await wa;

    co_return 1;
}
