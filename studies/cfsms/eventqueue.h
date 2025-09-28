/**
 * @file eventqueue.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#pragma once

#include <queue>
#include <optional>

template<typename TYPE>
class EventQueue
{
public:
    std::optional<TYPE> get()
    {
        if (!q.empty())
        {
            TYPE val = q.front();
            q.pop();
            return val;
        }
        return {};
    }

    void push(TYPE op)
    {
        q.push(op);
    }

private:
    std::queue<TYPE> q;
};
