/**
 * @file timer02.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer02.h"

#include <corolib/when_all.h>

/**
 * @brief Timer02::Timer02
 * @param ioContext
 */
Timer02::Timer02(
    boost::asio::io_context& ioContext)
    : m_ioContext(ioContext)
{
    print(PRI1, "Timer02::Timer02(...)\n");
}

/**
 * @brief Timer02::start
 */
void Timer02::start()
{
    mainTask();
}

/**
 * @brief Timer02::start_timer
 * @param idx
 * @param tmr
 * @param ms
 */
void Timer02::start_timer(async_operation_base& async_op, steady_timer& tmr, int ms)
{
	async_operation_base* p_async_op = &async_op;
	
    print(PRI1, "%p: Timer02::start_timer(%p, tmr, %d)\n", this, p_async_op, ms);

    tmr.expires_after(std::chrono::milliseconds(ms));

    tmr.async_wait(
        [this, p_async_op, ms](const boost::system::error_code& error)
        {
            print(PRI1, "%p: Timer02::handle_timer(): p_async_op = %p, ms = %d\n", this, p_async_op, ms);
            
            if (!error)
            {
                completionHandler_v(p_async_op);
            }
            else
            {
                print(PRI1, "%p: Timer02::handle_timer(): p_async_op = %p, ms = %d, error on timer: %s\n", this, p_async_op, ms, error.message().c_str());
                //stop();
            }
        });
}

/**
 * @brief Timer02::timerTask01
 * @return
 */
async_task<int> Timer02::timerTask01()
{
    steady_timer timer1(m_ioContext);

    print(PRI1, "%p: Timer02::timerTask01\n", this);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);
	
    start_timer(op_timer1, timer1, 1000);
    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 1000\n");

    for (int i = 0; i < 3; i++)
    {
        start_timer(op_timer1, timer1, 1000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 1000\n");

        start_timer(op_timer1, timer1, 2000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 2000\n");

        start_timer(op_timer1, timer1, 3000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 3000\n");

        start_timer(op_timer1, timer1, 4000);
        co_await op_timer1;
        print(PRI2, "--- timerTask01: after co_await op_timer1 --- 4000\n");

        start_timer(op_timer1, timer1, 5000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 5000\n");
    }

    co_return 1;
}

/**
 * @brief Timer02::timerTask02
 * @return
 */
async_task<int> Timer02::timerTask02()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);

    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer2{ this };
    op_timer2.auto_reset(true);
	
    start_timer(op_timer1, timer1, 1000);
    start_timer(op_timer2, timer2, 1000);

    co_await op_timer1;
    print(PRI1, "--- timerTask02: after co_await op_timer1 --- 1000\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask02: after co_await op_timer2 --- 1000\n");

    for (int i = 0; i < 3; i++)
    {
        start_timer(op_timer1, timer1, 1000);
        start_timer(op_timer2, timer2, 1500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 1000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 1500\n");

        start_timer(op_timer1, timer1, 2000);
        start_timer(op_timer2, timer2, 2500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 2000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 2500\n");

        start_timer(op_timer1, timer1, 3000);
        start_timer(op_timer2, timer2, 3500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 3000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 3500\n");

        start_timer(op_timer1, timer1, 4000);
        start_timer(op_timer2, timer2, 4500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 4000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 4500\n");

        start_timer(op_timer1, timer1, 5000);
        start_timer(op_timer2, timer2, 5500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 5000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 5500\n");
    }

    co_return 1;
}

/**
 * @brief Timer02::timerTask03
 * @return
 */
async_task<int> Timer02::timerTask03()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);
    
    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer2{ this };
    op_timer2.auto_reset(true);
	
    start_timer(op_timer1, timer1, 1000);
    start_timer(op_timer2, timer2, 1000);

    co_await op_timer1;
    print(PRI1, "--- timerTask03: after co_await op_timer1 --- 1000\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask03: after co_await op_timer2 --- 1000\n");

    for (int i = 0; i < 3; i++)
    {
        start_timer(op_timer1, timer1, 1000);
        start_timer(op_timer2, timer2, 1500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 1500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 1000\n");

        start_timer(op_timer1, timer1, 2000);
        start_timer(op_timer2, timer2, 2500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 2500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 2000\n");
        
        start_timer(op_timer1, timer1, 3000);
        start_timer(op_timer2, timer2, 3500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 3500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 3000\n");
       
        start_timer(op_timer1, timer1, 4000);
        start_timer(op_timer2, timer2, 4500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 4500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 4000\n");
       
        start_timer(op_timer1, timer1, 5000);
        start_timer(op_timer2, timer2, 5500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 5500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 5000\n");
    }

    co_return 1;
}

/**
 * @brief Timer02::timerTask04
 * @return
 */
async_task<int> Timer02::timerTask04()
{
    steady_timer timer1(m_ioContext);
    steady_timer timer2(m_ioContext);

    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer1{ this };
    op_timer1.auto_reset(true);

    print(PRI1, "%p: Timer02::timerTask02\n", this);
    async_operation<void> op_timer2{ this };
    op_timer2.auto_reset(true);
	
    start_timer(op_timer1, timer1, 1000);
    start_timer(op_timer2, timer2, 1000);

    when_all<async_operation<void>> wa({ &op_timer1, &op_timer2 });
    co_await wa;
    print(PRI1, "--- timerTask04: after co_await wa --- 1000 + 1000\n");
	
    for (int i = 0; i < 3; i++)
    {
        start_timer(op_timer1, timer1, 1000);
        start_timer(op_timer2, timer2, 1500);

        when_all<async_operation<void>> wa1({ &op_timer1, &op_timer2 });
        co_await wa1;
        print(PRI1, "--- timerTask04: after co_await wa1 --- 1000 + 1500\n");
	
        start_timer(op_timer1, timer1, 2000);
        start_timer(op_timer2, timer2, 2500);

        when_all<async_operation<void>> wa2({ &op_timer1, &op_timer2 });
        co_await wa2;
        print(PRI1, "--- timerTask04: after co_await wa2 --- 2000 + 2500\n");
	
        start_timer(op_timer1, timer1, 3000);
        start_timer(op_timer2, timer2, 3500);

        when_all<async_operation<void>> wa3({ &op_timer1, &op_timer2 });
        co_await wa3;
        print(PRI1, "--- timerTask04: after co_await wa3 --- 3000 + 3500\n");
	
        start_timer(op_timer1, timer1, 4000);
        start_timer(op_timer2, timer2, 4500);

        when_all<async_operation<void>> wa4({ &op_timer1, &op_timer2 });
        co_await wa4;
        print(PRI1, "--- timerTask04: after co_await wa4 --- 4000 + 4500\n");
	
        start_timer(op_timer1, timer1, 5000);
        start_timer(op_timer2, timer2, 5500);

        when_all<async_operation<void>> wa5({ &op_timer1, &op_timer2 });
        co_await wa5;
        print(PRI1, "--- timerTask04: after co_await wa5 --- 5000 + 5500\n");
    }

    co_return 1;
}

/**
 * @brief Timer02::mainTask
 * @return
 */
async_task<int> Timer02::mainTask()
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
