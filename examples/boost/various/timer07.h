/**
 * @file timer07.h
 * @brief This example is a simplication of a server application
 * that starts a coroutine each time it accepts a connection from a client.
 * Each coroutine instances handles the communication with a single client.
 * 
 * The acceptance of a client connection is simulated by the expiration of a timer.
 * Each time the timer expires, the client-handling coroutine is started.
 * After NR_TASKS timer expirations and, consequently NR_TASKS started coroutines,
 * the server application co_awaits the completion of the started coroutines,
 * before it starts accepting client connections again (i.e., waits for the expiration of the timer).
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER07_H
#define TIMER07_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/commservice.h>

using namespace corolib;

class Timer07 : public CommService
{
public:
    Timer07(boost::asio::io_context& ioContext);
    async_task<int> mainTask();

protected:
    async_operation<void> start_timer(steady_timer& timer, int ms);
    void start_timer_impl(const int idx, steady_timer& tmr, int ms);

    void start_timer(async_operation_base& async_op, steady_timer& tmr, int ms);

    async_task<int> clientTask(int i);
private:
    boost::asio::io_context& m_ioContext;
    bool m_running;
};

#endif
