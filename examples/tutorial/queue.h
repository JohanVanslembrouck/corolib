/**
 * @file queue.h
 * @brief
 * This version should be used in single-threaded applications only.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <queue>

template <typename TYPE>
class Queue
{
public:
    void push(TYPE&& op)
    {
        q.push(std::move(op));
    }

    bool empty()
    {
        return q.empty();
    }

    TYPE pop()
    {
        TYPE op = q.front();
        q.pop();
        return op;
    }

protected:
    std::queue<TYPE> q;
};

#endif
