/**
 * @file timer02.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef TIMER02_H
#define TIMER02_H

#include <boost/asio/steady_timer.hpp>

using boost::asio::steady_timer;

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/commservice.h>

using namespace corolib;

class Timer02 : public CommService
{
public:
	Timer02(boost::asio::io_context& ioContext);
	void start();

protected:
	void start_timer(async_operation_base& async_op, steady_timer& tmr, int ms);

	async_task<int> mainTask();
	async_task<int> timerTask01();
	async_task<int> timerTask02();
	async_task<int> timerTask03();
	async_task<int> timerTask04();

private:
	boost::asio::io_context& m_ioContext;
};

#endif
