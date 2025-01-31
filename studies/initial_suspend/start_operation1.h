/**
 * @file start_opertion1.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _START_OPERATION1_H_
#define _START_OPERATION1_H_

#include "print.h"
#include "mini_awaiter_ts.h"

void start_operation1(mini_awaiter_ts& ma, bool delayafterstart = false) {
    std::thread thread1([&ma]() {
        print(PRI1, "start_operation1(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print(PRI1, "start_operation1(): thread1: ma.set_result_and_resume(10);\n");
        ma.set_result_and_resume(10);
        print(PRI1, "start_operation1(): thread1: return;\n");
        });
    thread1.detach();

    if (delayafterstart) {
        print(PRI1, "start_operation(): std::this_thread::sleep_for(std::chrono::milliseconds(1500));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
}

#endif
