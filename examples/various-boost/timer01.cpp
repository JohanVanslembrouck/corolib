/**
 * @file timer01.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer01.h"
#include <chrono>

#include <corolib/when_all.h>

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

/**
 * @brief Timer01::start_timer
 * @param timer
 * @param ms
 */
async_operation<void> Timer01::start_timer(steady_timer& timer, int ms)
{
    int index = get_free_index();
    print(PRI1, "%p: Timer01::start_timer(timer, %d): index = %d\n", this, ms, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms);
    return ret;
}

/**
 * @brief Timer01::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer01::start_timer_impl(const int idx, steady_timer& tmr, int ms)
{
    print(PRI1, "%p: Timer01::start_timer_impl(%d, tmr, %d)\n", this, idx, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, idx, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer01::handle_timer(): idx = %d, ms = %d\n", this, idx, ms);
            
            if (!error)
            {
                completionHandler_v(idx);
            }
            else
            {
                print(PRI1, "%p: Timer01::handle_timer(): idx = %d, ms = %d, error on timer: %s\n", this, idx, ms, error.message().c_str());
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

    async_operation<void> op_timer1 = start_timer(timer1, 1000);
    op_timer1.auto_reset(true);
    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 1000\n");

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

    async_operation<void> op_timer1 = start_timer(timer1, 1000);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 1000);
    op_timer2.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask02: after co_await op_timer1 --- 1000\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask02: after co_await op_timer2 --- 1000\n");

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
        async_operation<void> op_timer2d = start_timer(timer2, 4500);

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

    async_operation<void> op_timer1 = start_timer(timer1, 1000);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 1000);
    op_timer2.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask03: after co_await op_timer1 --- 1000\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask03: after co_await op_timer2 --- 1000\n");

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
 * @brief Timer01::timerTask04
 * @return
 */
async_task<int> Timer01::timerTask04()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    async_operation<void> op_timer1 = start_timer(timer1, 1000);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 1000);
    op_timer2.auto_reset(true);

    when_all<async_operation<void>> wa({ &op_timer1, &op_timer2 });
    co_await wa;
    print(PRI1, "--- timerTask04: after co_await wa --- 1000 + 1000\n");

    for (int i = 0; i < 3; i++)
    {
        async_operation<void> op_timer1a = start_timer(timer1, 1000);
        async_operation<void> op_timer2a = start_timer(timer2, 1500);

        when_all<async_operation<void>> wa1({ &op_timer1a, &op_timer2a });
        co_await wa1;
        print(PRI1, "--- timerTask04: after co_await wa1 --- 1000 + 1500\n");

        async_operation<void> op_timer1b = start_timer(timer1, 2000);
        async_operation<void> op_timer2b = start_timer(timer2, 2500);
        
        when_all<async_operation<void>> wa2({ &op_timer1b, &op_timer2b });
        co_await wa2;
        print(PRI1, "--- timerTask04: after co_await wa2 --- 2000 + 2500\n");
    
        async_operation<void> op_timer1c = start_timer(timer1, 3000);
        async_operation<void> op_timer2c = start_timer(timer2, 3500);
       
        when_all<async_operation<void>> wa3({ &op_timer1c, &op_timer2c });
        co_await wa3;
        print(PRI1, "--- timerTask04: after co_await wa3 --- 3000 + 3500\n");
        
        async_operation<void> op_timer1d = start_timer(timer1, 4000);
        async_operation<void> op_timer2d = start_timer(timer2, 4500);

        when_all<async_operation<void>> wa4({ &op_timer1d, &op_timer2d });
        co_await wa4;
        print(PRI1, "--- timerTask04: after co_await wa4 --- 4000 + 4500\n");
        
        async_operation<void> op_timer1e = start_timer(timer1, 5000);
        async_operation<void> op_timer2e = start_timer(timer2, 5500);

        when_all<async_operation<void>> wa5({ &op_timer1e, &op_timer2e });
        co_await wa5;
        print(PRI1, "--- timerTask04: after co_await wa5 --- 5000 + 5500\n");
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
	async_task<int> t4 = timerTask04();

    print(PRI1, "--- mainTask: when_all<async_task<int>> wa({ &t1, &t2, &t3, &t4 });\n");
    when_all<async_task<int>> wa({ &t1, &t2, &t3, &t4 });
    print(PRI1, "--- mainTask: co_await wa;\n");
    co_await wa;

    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 1;
}
