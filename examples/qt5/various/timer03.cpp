/**
 * @file timer03.cpp
 * @brief This example shows how one instance of an async_operation<void> object
 * can be used to resume several coroutines that co_wait this object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <QThread>
#include <QDebug>

#include <corolib/when_all.h>

#include "timer03.h"

/**
 * @brief Timer03::Timer03
 * @param parent
 */
Timer03::Timer03(QObject *parent)
    : QObject(parent)
    , m_running(true)
{
    qDebug() << Q_FUNC_INFO;
}

/**
 * @brief Timer03::start
 * called from main() after having created a Timer03 object
 */
void Timer03::start()
{
    qInfo() << Q_FUNC_INFO;
  
    mainTask();
}

// Coroutine related
// =================

void Timer03::connect_to_timer(async_operation_base& async_op, QTimer& tmr, QMetaObject::Connection& conn, bool doDisconnect)
{
    async_operation_base* p_async_op = &async_op;
    QMetaObject::Connection* p_conn = &conn;
    
    print(PRI1, "%p: Timer03::connect_to_timer()\n");

    conn = connect(&tmr, &QTimer::timeout,
        [this, p_async_op, p_conn, doDisconnect]()
        {
            print(PRI1, "%p: Timer03::handle_timer() lambda\n", this);

            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(p_async_op);

            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
            else
            {
                // This can occur when the async_operation_base has gone out of scope.
                print(PRI1, "%p: Timer03::handle_timer(): Warning: p_async_op == nullptr\n", this);
            }
            if (doDisconnect)
            {
                print(PRI1, "%p: Timer03::handle_timer(): disconnecting\n", this);
                if (!disconnect(*p_conn))
                {
                    print(PRI1, "%p: Timer03::handle_timer(): Warning: disconnect failed\n", this);
                }
            }
        }
    );
}

// Using coroutines
// ================

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
 * @brief Timer03::mainTask
 * @return
 */
async_task<int> Timer03::mainTask()
{
    qDebug() << Q_FUNC_INFO;
    
    qDebug() << Q_FUNC_INFO << "begin";

    QMetaObject::Connection conn1;
    QTimer timer1(this);
    timer1.setSingleShot(true);
    
    QMetaObject::Connection conn2;
    QTimer timer2(this);
    timer2.setSingleShot(true);

    async_operation<void> op_timer1(this);
    op_timer1.auto_reset(true);
    async_operation<void> op_timer2(this);
    op_timer2.auto_reset(true);

    connect_to_timer(op_timer1, timer1, conn1); 
    connect_to_timer(op_timer2, timer2, conn2); 

    async_task<int> t1a = timerTask01a(op_timer1);
    async_task<int> t1b = timerTask01b(op_timer1);
    async_task<int> t1c = timerTask01c(op_timer1);
    
    timer1.start(1000);
    timer2.start(1500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 1500\n", this);

    timer1.start(2000);
    timer2.start(2500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 2500\n", this);

    timer1.start(3000);
    timer2.start(3500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 3500\n", this);

    timer1.start(4000);
    timer2.start(4500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 4500\n", this);

    timer1.start(5000);
    timer2.start(5500);
    co_await op_timer2;
    print(PRI1, "%p: Timer03::mainTask: after co_await op_timer2 5500\n", this);

    // Set m_running to false to make the coroutines leave their loop.
    m_running = false;
    
    // Start the timer: when it expires, the 3 coroutines will leave their loop.
    timer1.start(1000);
    
    print(PRI1, "--- mainTask: when_all<async_task<int>> wa({ &t1a, &t1b, &t1c });\n");
    when_all<async_task<int>> wa({ &t1a, &t1b, &t1c });
    print(PRI1, "--- mainTask: co_await wa;\n");
    co_await wa;

    print(PRI1, "--- mainTask: co_return 0;\n");
    co_return 0;
}
