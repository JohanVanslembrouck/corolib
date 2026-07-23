/**
 * @file p3000_async.h
 * @brief This file defines (very simplified) asynchronous file operations.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P3000_ASYNC_H_
#define _P3000_ASYNC_H_

#include <functional>

#include "runeventqueue.h"
#include "print.h"

extern EventQueueThrFunctionX evqueuethr;

/**
 * @brief Create a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_create(FunctionVoidBool functor) {
    print(PRI1, "async_create()\n");
    evqueuethr.push(functor);
}

/**
 * @brief Open a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_open(FunctionVoidInt functor) {
    print(PRI1, "async_open()\n");
    evqueuethr.push(functor);
}

/**
 * @brief Write to a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_write(char* , FunctionVoidInt functor) {
    print(PRI1, "async_write()\n");
    evqueuethr.push(functor);
}

/**
 * @brief Read from a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_read(FunctionVoidString functor) {
    print(PRI1, "async_read()\n");
    evqueuethr.push(functor);
}

/**
 * @brief Close a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_close(FunctionVoidBool functor) {
    print(PRI1, "async_close()\n");
    evqueuethr.push(functor);
}

/**
 * @brief Remove a file asynchronously and push the functor that will be called when the reply arrives onto evqueuethr.
 */
void async_remove(FunctionVoidBool functor) {
    print(PRI1, "async_remove()\n");
    evqueuethr.push(functor);
}

void async_remove2(FunctionVoidVoid functor) {
    print(PRI1, "async_remove()\n");
    evqueuethr.push(functor);
}

#endif
