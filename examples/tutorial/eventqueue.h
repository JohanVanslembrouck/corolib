/**
 * @file eventqueue.h
 * @brief
 * EventQueue is a very simplified version of an event queue that is used
 * in communication frameworks.
 *
 * Applications will register a function or functor with an event or with a file or socket (descriptor).
 * When an event occurs, or when there is activity on the file/socket,
 * the communication framework will call the associated function/functor from an event loop.
 * This function is thus used as a callback function.
 * The application should regularly return control to the event loop so it can 
 * run over the events that have arrived (and be placed in the event queue).
 *
 * The events are usually generated from an interrupt service routine (ISR), followed
 * by a deferred procedure call (DPC) or a workqueue element.
 * Those last are responsible for placing the event in the event queue.
 *
 * In this simplified implementation, the functors themselves are placed in the 
 * event queue by the application calling the push() member function.
 * The event loop (member function run()) runs over all elements in the event queue
 * and calls the functor using the call operator.
 * 
 * This version should be used in single-threaded applications only.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _EVENTQUEUE_H_
#define _EVENTQUEUE_H_

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

#include <functional>

using EventQueueFunctionVoidInt = Queue<std::function<void(int)>>;
using EventQueueFunctionVoidVoid = Queue<std::function<void(void)>>;

void runEventQueue(EventQueueFunctionVoidInt& queue, int val);
void runEventQueue(EventQueueFunctionVoidVoid& queue);

#endif
