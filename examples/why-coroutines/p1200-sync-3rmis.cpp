/**
 * @file p1200-sync-3rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

class Class01
{
public:
    void function1()
    {
        printf("Class01::function1()\n");
        int ret1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
        // 1 Do stuff
        if (ret1 == gval1) {
            int ret2 = remoteObj2.op2(gin21, gin22, gout21);
            // 2 Do stuff
        }
        else {
            int ret3 = remoteObj3.op3(gin31, gout31, gout32);
            // 3 Do stuff
        }
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01.function1(); });
    eventQueue.push([]() { class01.function1(); });
    eventQueue.run();
    return 0;
}
