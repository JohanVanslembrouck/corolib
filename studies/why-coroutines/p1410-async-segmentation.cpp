/**
 * @file p1410-async-segmentation.cpp
 * @brief This file contains an asynchronous implementation of p1400-sync-segmentation.cpp.
 * Notice how the code is very different from the synchronous implementation.
 *
 * Notice how this implementation is very similar to that in p1310-async-nested-loop.cpp.
 * Differences have been marked in the code.
 * 
 * @author Johan Vanslembrouck
 */

#include <stdio.h>
#include <chrono>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1400.h"              // difference with p1310-async-nested-loop.cpp

class Class01
{
private:
    struct function1_ctxt_t
    {
        ~function1_ctxt_t() { printf("function1_ctxt_t::~function1_ctxt_t()\n"); }

        Msg msg;
        int i = 0;
        int j = 0;
        int counter = 0;
        std::chrono::high_resolution_clock::time_point start_time;
    };

public:
    void function1()
    {
        function1_ctxt_t* ctxt = new function1_ctxt_t;

        printf("Class01::function1(): counter = %d\n", ctxt->counter);
        ctxt->start_time = std::chrono::high_resolution_clock::now();
        ctxt->msg = Msg(0);
        remoteObj1.sendc_op1(ctxt->msg,
            [this, ctxt](Msg msg) {                     // difference with p1310-async-nested-loop.cpp
                this->function1a(ctxt, msg);            // difference with p1310-async-nested-loop.cpp
            });
    }

protected:
    void function1a(function1_ctxt_t* ctxt, Msg /*msgout*/)         // difference with p1310-async-nested-loop.cpp
    {
        // Do something with msgout                                 // difference with p1310-async-nested-loop.cpp
        printf("Class01::function1a(Msg): counter = %d\n", ctxt->counter);
        if (ctxt->j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", ctxt->i, ctxt->j, ctxt->counter);
            remoteObj1.sendc_op1(ctxt->msg,
                        [this, ctxt](Msg msg) {                 // difference with p1310-async-nested-loop.cpp
                            this->function1a(ctxt, msg);        // difference with p1310-async-nested-loop.cpp
                        });
            ctxt->j++;
            ctxt->counter++;
        }
        else {
            // End of inner loop
            ctxt->j = 0;
            ctxt->i++;
            if (ctxt->i < MAX_MSG_LENGTH) {
                ctxt->msg = Msg(ctxt->i);               // difference with p1310-async-nested-loop.cpp
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", ctxt->i, ctxt->j, ctxt->counter);
                remoteObj1.sendc_op1(ctxt->msg,
                            [this, ctxt](Msg msg) {                 // difference with p1310-async-nested-loop.cpp
                                this->function1a(ctxt, msg);        // difference with p1310-async-nested-loop.cpp
                            });
                ctxt->j++;
                ctxt->counter++;
            }
            else {
                // End of inner and outer loop
                std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
                double time_taken =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - ctxt->start_time).count();
                printf("Class01::function1a(): time_taken = %f s\n", time_taken / 1000000000.0);
                delete ctxt;
            }
        }
    }

private:
    RemoteObjectImpl remoteObjImpl;                 // difference with p1310-async-nested-loop.cpp
    RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1310-async-nested-loop.cpp
};

EventQueue eventQueue;

int main()
{
    printf("main(): begin\n");
    Class01 class01;
    class01.function1();
    class01.function1();
    eventQueue.run();
    printf("main(): end\n");
    return 0;
}
