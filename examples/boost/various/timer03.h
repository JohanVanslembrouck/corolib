/**
 * @file timer03.h
 * @brief This example shows how one instance of an async_operation<void> object
 * can be used to resume several coroutines that co_wait this object.
 * Prerequisite: call resume_multiple_coroutines(true); in main().
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER03_H
#define TIMER03_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/commservice.h>

using namespace corolib;

class Timer03 : public CommService
{
public:
    Timer03(boost::asio::io_context& ioContext);
    async_task<int> mainTask();

protected:
    void start_timer(async_operation_base& async_op, steady_timer& tmr, int ms);

    async_task<int> timerTask01a(async_operation<void>& op_tmr);
    async_task<int> timerTask01b(async_operation<void>& op_tmr);
    async_task<int> timerTask01c(async_operation<void>& op_tmr);

private:
    boost::asio::io_context& m_ioContext;
    bool m_running;
};

#endif
