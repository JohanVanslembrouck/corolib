/**
 * @file timer03.h
 * @brief This example shows how one instance of an async_operation<void> object
 * can be used to resume several coroutines that co_wait this object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER03_H
#define TIMER03_H

#include <QObject>
#include <QTimer>

#include <corolib/async_operation.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>

using namespace corolib;

class Timer03 : public QObject, public CommService
{
    Q_OBJECT

public:
    explicit Timer03(QObject *parent = nullptr);
    async_task<int> mainTask();

protected:
    void connect_to_timer(async_operation_base& async_op, QTimer& tmr, QMetaObject::Connection& conn, bool doDisconnect = false);
   
    async_task<int> timerTask01a(async_operation<void>& op_tmr);
    async_task<int> timerTask01b(async_operation<void>& op_tmr);
    async_task<int> timerTask01c(async_operation<void>& op_tmr);

private:
    bool m_running;
};

#endif // TIMER03_H
