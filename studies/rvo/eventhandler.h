/**
 * @file eventhandler.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <functional>

#ifndef _EVENTHANDLER_H_
#define _EVENTHANDLER_H_

/**
 * @brief EventHandler is a very simple event handler that has room
 * for a single event-handling function at a time.
 * It will (should) only be used in applications that start one operation.
 *
 */
class EventHandler
{
public:
    void set(std::function<void(int)>&& fnt, int in) {
        printf("EventHandler::set(..., %d)\n", in);
        eventHdlr = std::move(fnt);
        input = in;
        inUse = true;
    }

    void run() {
        printf("EventHandler::run()\n");
        if (inUse) {
            eventHdlr(input);
            inUse = false;
        }
    }

private:
    int input;
    std::function<void(int)> eventHdlr;
    bool inUse = false;
};

extern EventHandler eventHandler;

#endif
