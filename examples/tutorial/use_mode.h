/**
 * @file use_mode.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef USE_MODE_H
#define USE_MODE_H

/**
 * Enum used to simulate the completion of an asynchronous operation:
 * 1) USE_NONE: The application (function main() or a function called from main())
 *    has to call completionHandler "manually".
 * 2) USE_EVENTQUEUE: places the completionHandler in an eventQueue.
 *    The application (function main() or a function called from main())
 *    will run the event loop to call the completionHandler(s) in the eventQueue.
 * 3) USE_THREAD: starts a detached thread that will
 *    call the completionHandler (after a delay of X ms).
 * 4) USE_THREAD_QUEUE: starts a detached thread that will
 *    place the completionHandler in a queue (after the delay of X ms).
 *    The application (function main() or a function called from main()) 
 *    will run the completionHandler in its own context.
 * 5) USE_IMMEDIATE_COMPLETION: calls the completionHandler immediately.
 */

enum class UseMode
{
    USE_NONE,
    USE_EVENTQUEUE,
    USE_THREAD,
    USE_THREAD_QUEUE,
    USE_IMMEDIATE_COMPLETION
};

const int defaultCompletionValue = 10;

#endif
