/**
 * @file timer01.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER01_H
#define TIMER01_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/commservice.h>

using namespace corolib;

class Timer01 : public CommService
{
public:
	Timer01(boost::asio::io_context& ioContext);
	void start();

protected:
	async_operation<void> start_timer(steady_timer& timer, int ms);
	void start_timer_impl(const int idx, steady_timer& tmr, int ms);

	async_task<int> mainTask();
	async_task<int> timerTask01();
	async_task<int> timerTask02();
	async_task<int> timerTask03();
	async_task<int> timerTask04();

private:
	boost::asio::io_context& m_ioContext;
};

#endif
