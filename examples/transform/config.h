/**
 *  Filename: config.h
 *  Description:
 *  This file allows selecting between the coroutine header file (coroutine) that comes with the C++ compiler
 *  and the header file originally written by Lewis Baker (lbcoroutine.h) as part of the code in https://godbolt.org/z/xaj3Yxabn
 * 
 *  The selection is usually done from CMakeLists.txt, but can be done from a .cpp file as well.
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
