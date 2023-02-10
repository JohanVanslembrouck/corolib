/**
 * @file p2100.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P2100_H_
#define _P2100_H_

#include "class01.h"

void completionflow();
int task1(Class01&& object01);

extern thread_local Class01 thr_object01;

#endif
