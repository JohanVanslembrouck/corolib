/**
 *  Filename: class01.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#ifndef _CLASS01_H_
#define _CLASS01_H_

#include <functional>

#include "commservice.h"
#include "async_operation.h"

#include "eventqueue.h"

extern EventQueue eventQueue;

using namespace corolib;

enum UseMode
{
	USE_NONE,
	USE_EVENTQUEUE,
	USE_THREAD
};

class Class01 : public CommService
{
public:
	Class01(UseMode useMode = USE_NONE) 
		: m_useMode(useMode)
	{
	}
	
	async_operation<int> start_operation();
	void start_op(const int idx);

	std::function<void(int)> operation;
	
private:
	UseMode	m_useMode;
};

#endif
