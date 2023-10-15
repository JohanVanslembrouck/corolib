/**
 *  Filename: config.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef __MYCONFIG_H_
#define __MYCONFIG_H_

#if USE_LBCOROUTINE
#include "lbcoroutine.h"
#else
#include <coroutine>
#endif

#endif // __MYCONFIG_H_
