/**
 * @file eventqueue.h
 * @brief
 * See the event queue description in tutorial/eventqueue.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _EVENTQUEUE_H_
#define _EVENTQUEUE_H_

#include <stdio.h>

#include <functional>
#include <queue>
#include <thread>

class EventQueue
{
public:
    void run()
    {
        while (!q.empty())
        {
            //printf("EventQueue::run(): std::this_thread::sleep_for(std::chrono::milliseconds(100));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::function<void(void)> op = q.front();
            q.pop();
            op();
        }
    }

    void push(std::function<void(void)> op)
    {
        q.push(op);
    }

private:
    std::queue<std::function<void(void)>> q;
};

#endif
