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
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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
    /**
	 * @brief run takes a functor from the front of the internal queue of void(int) functors
	 * and calls this functor with argument = 10.
	 * run uses a delay of 1000 ms before each functor invocation.
	 */
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
    
	/**
	 * @brief push places a functor in the internal queue.
	 */
    void push(std::function<void(int)> op)
    {
        q.push(op);
    }
    
private:
    std::queue<std::function<void(int)>> q;
};

#endif
