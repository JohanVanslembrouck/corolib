/**
 *  Filename: configvf.h
 *  Description:
 *  This file allows selecting between the coroutine header file (coroutine) that comes with the C++ compiler
 *  and the header file lbcoroutinevf.h
 * 
 *  The selection is usually done from CMakeLists.txt, but can be done from a .cpp file as well.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef __MYCONFIGVF_H_
#define __MYCONFIGVF_H_

#if USE_LBCOROUTINE
#include "lbcoroutinevf.h"
#else
#include <coroutine>
#endif

#endif // __MYCONFIG_VF_H_
