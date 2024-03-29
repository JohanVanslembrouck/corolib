/**
 * @file timer01.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QThread>
#include <QDebug>
#include <QCoreApplication>

#include <corolib/when_all.h>

#include "timer01.h"

/**
 * @brief Timer01::Timer01
 * @param parent
 */
Timer01::Timer01(QObject *parent)
    : QObject(parent)
{
    qDebug() << Q_FUNC_INFO;
}

// Coroutine related
// =================

/**
 * @brief Timer01::start_timer starts a timer.
 * This function must only be called to create and return an async_operation<void> object
 * that afterwards can be reused.
 * The associated Qt timer must then be restarted using timerX.start(timeout);
 *
 * @param timer
 * @param ms
 * @param doDisconnect
 * @return
 */
async_operation<void> Timer01::start_timer(QTimer& timer, int ms, bool doDisconnect)
{
    int index = get_free_index();
    print(PRI1, "%p: Timer01::start_timer(): index = %d\n", this, index);
    async_operation<void> ret{ this, index };
    start_timer_impl(index, timer, ms, doDisconnect);
    return ret;
}

/**
 * @brief Timer01::start_timer_impl
 * @param idx
 * @param tmr
 * @param ms
 * @param doDisconnect
 */
void Timer01::start_timer_impl(const int idx, QTimer& tmr, int ms, bool doDisconnect)
{
    print(PRI1, "%p: Timer01::start_timer_impl(): idx = %d, operation = %p\n", this, idx, get_async_operation(idx));

    tmr.start(ms);

    m_connections[idx] = connect(&tmr, &QTimer::timeout,
        [this, idx, doDisconnect]()
        {
            print(PRI1, "%p: Timer01::handle_timer() lambda: idx = %d\n", this, idx);

            async_operation_base* om_async_operation = get_async_operation(idx);
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: Timer01::handle_timer(): idx = %d, Warning: om_async_operation_t == nullptr\n", this, idx);
            }
            if (doDisconnect)
            {
                print(PRI1, "%p: Timer01::handle_timer(): idx = %d, disconnecting\n", this, idx);
                if (!disconnect(m_connections[idx]))
                {
                    print(PRI1, "%p: Timer01::handle_timer(): idx = %d, Warning: disconnect failed\n", this, idx);
                }
            }
        }
    );
}

// Using coroutines
// ================

/**
 * @brief Timer01::timerTask01
 * @return
 */
async_task<int> Timer01::timerTask01()
{
    qDebug() << Q_FUNC_INFO << "begin";

    QTimer timer1(this);
    timer1.setSingleShot(true);

    async_operation<void> op_timer1 = start_timer(timer1, 500);
    op_timer1.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 500\n");

    for (int i = 0; i < 3; i++)
    {
        print(PRI1, "--- timerTask01: i = %d\n", i);

        timer1.start(1000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 1000\n");

        timer1.start(2000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 2000\n");

        timer1.start(3000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 3000\n");

        timer1.start(4000);
        co_await op_timer1;
        print(PRI2, "--- timerTask01: after co_await op_timer1 --- 4000\n");

        timer1.start(5000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 5000\n");
    }

    qDebug() << Q_FUNC_INFO << "end";
    co_return 1;
}

/**
 * @brief Timer01::timerTask02
 * @return
 */
async_task<int> Timer01::timerTask02()
{
    qDebug() << Q_FUNC_INFO << "begin";

    QTimer timer1(this);
    timer1.setSingleShot(true);
    QTimer timer2(this);
    timer2.setSingleShot(true);

    async_operation<void> op_timer1 = start_timer(timer1, 500);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 550);
    op_timer2.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask02: after co_await op_timer1 --- 500\n");
    co_await op_timer2;
    print(PRI1, "--- timerTask02: after co_await op_timer2 --- 550\n");

    for (int i = 0; i < 3; i++)
    {
        print(PRI1, "--- timerTask02: i = %d\n", i);

        timer1.start(1000);
        timer2.start(1500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 1000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 1500\n");

        timer1.start(2000);
        timer2.start(2500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 2000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 2500\n");

        timer1.start(3000);
        timer2.start(3500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 3000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 3500\n");

        timer1.start(4000);
        timer2.start(4500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 4000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 4500\n");

        timer1.start(5000);
        timer2.start(5500);

        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 5000\n");
        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 5500\n");
    }

    qDebug() << Q_FUNC_INFO << "end";
    co_return 1;
}

/**
 * @brief Timer01::timerTask03
 * @return
 */
async_task<int> Timer01::timerTask03()
{
    qDebug() << Q_FUNC_INFO << "begin";

    QTimer timer1(this);
    timer1.setSingleShot(true);
    QTimer timer2(this);
    timer2.setSingleShot(true);

    async_operation<void> op_timer1 = start_timer(timer1, 500);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2 = start_timer(timer2, 550);
    op_timer2.auto_reset(true);

    co_await op_timer2;
    print(PRI1, "--- timerTask03: after co_await op_timer2 --- 550\n");
    co_await op_timer1;
    print(PRI1, "--- timerTask03: after co_await op_timer1 --- 500\n");

    for (int i = 0; i < 3; i++)
    {
        print(PRI1, "--- timerTask03: i = %d\n", i);

        timer1.start(1000);
        timer2.start(1500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 1500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 1000\n");

        timer1.start(2000);
        timer2.start(2500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 2500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 2000\n");

        timer1.start(3000);
        timer2.start(3500);

        co_await op_timer2;
        print(PRI1, "--- timerTask02: after co_await op_timer2 --- 3500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask02: after co_await op_timer1 --- 3000\n");

        timer1.start(4000);
        timer2.start(4500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 4500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 4000\n");

        timer1.start(5000);
        timer2.start(5500);

        co_await op_timer2;
        print(PRI1, "--- timerTask03: after co_await op_timer2 --- 5500\n");
        co_await op_timer1;
        print(PRI1, "--- timerTask03: after co_await op_timer1 --- 5000\n");
    }

    qDebug() << Q_FUNC_INFO << "end";
    co_return 1;
}

/**
 * @brief Timer01::timerTask04
 * @return
 */
async_task<int> Timer01::timerTask04()
{
    qDebug() << Q_FUNC_INFO << "begin";

    QTimer timer1(this);
    timer1.setSingleShot(true);
    QTimer timer2(this);
    timer2.setSingleShot(true);

    async_operation<void> op_timer1 = start_timer(timer1, 500);
    op_timer1.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask04: after co_await op_timer1 --- 500\n");

    for (int i = 0; i < 5; i++)
    {
        print(PRI1, "--- timerTask04: i = %d\n", i);

        print(PRI1, "--- timerTask04: --------------------------- \n");
        timer1.start(1000);
        co_await op_timer1;
        print(PRI1, "--- timerTask04: after co_await op_timer1 --- 1000\n");
#if 1
        async_operation<void> op_timer2 = start_timer(timer2, 2000, true);
        op_timer2.auto_reset(true);
        co_await op_timer2;
        print(PRI1, "--- timerTask04: after co_await op_timer2 --- 2000\n");
#endif
        timer1.start(5000);
        co_await op_timer1;
        print(PRI1, "--- timerTask04: after co_await op_timer1 --- 5000\n");
    }

    qDebug() << Q_FUNC_INFO << "end";
    co_return 1;
}

/**
 * @brief Timer01::mainTask
 * @return
 */
async_task<int> Timer01::mainTask()
{
    qDebug() << Q_FUNC_INFO;

    async_task<int> t1 = timerTask01();
    async_task<int> t2 = timerTask02();
    async_task<int> t3 = timerTask03();
    async_task<int> t4 = timerTask04();

    print(PRI1, "--- mainTask: when_all wa({ &t1, &t2, &t3, &t4 });\n");
    when_all wa({ &t1, &t2, &t3, &t4 });
    print(PRI1, "--- mainTask: co_await wa;\n");
    co_await wa;

    QCoreApplication::quit();
    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 0;
}
