/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
 #include "eventqueuethr.h"
 
void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size)
{
    for (int i = 0; i < size; i++)
    {
        std::function<void(int)> fun = queue.pop();
        fun(10);
    }
}
