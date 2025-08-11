/**
 * @file p1310-async-nested-loop.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <chrono>

#include "common.h"
#include "eventqueue.h"
#include "buf+msg.h"

#include "p1300.h"

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
            [this, ctxt]() {
                this->function1a(ctxt);
            });
    }

protected:
    void function1a(function1_ctxt_t* ctxt)
    {
        printf("Class01::function1a(): counter = %d\n", ctxt->counter);
        if (ctxt->j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", ctxt->i, ctxt->j, ctxt->counter);
            remoteObj1.sendc_op1(ctxt->msg,
                        [this, ctxt]() {
                            this->function1a(ctxt);
                        });
            ctxt->j++;
            ctxt->counter++;
        }
        else {
            ctxt->j = 0;
            ctxt->i++;
            if (ctxt->i < MAX_MSG_LENGTH) {
                ctxt->msg = Msg(ctxt->i * 10);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", ctxt->i, ctxt->j, ctxt->counter);
                remoteObj1.sendc_op1(ctxt->msg,
                            [this, ctxt]() {
                                this->function1a(ctxt);
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
    RemoteObject1 remoteObj1;
};

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    Class01 class01;
    class01.function1();
    class01.function1();
    eventQueue.run();
    return 0;
}
