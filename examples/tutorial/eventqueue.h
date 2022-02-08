/**
 *  Filename: eventqueue.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#ifndef _EVENTQUEUE_H_
#define _EVENTQUEUE_H_

#include <functional>
#include <queue>
#include <thread>

#include <corolib/print.h>

using namespace corolib;

class EventQueue
{
public:
    void run()
    {
        while (!q.empty())
        {
            print(PRI1, "EventQueue::run(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
            std::function<void(int)> op = q.front();
            op(10);
            q.pop();
        }
    }
    
    void push(std::function<void(int)> op)
    {
        q.push(op);
    }
    
private:
    std::queue<std::function<void(int)>> q;
};

#endif
