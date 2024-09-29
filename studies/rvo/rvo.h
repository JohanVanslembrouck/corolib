/**
 * @file rvo.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _RVO_H_
#define _RVO_H_

#include <functional>

void async_op(int i, std::function<void(int)>&& op);

#endif
