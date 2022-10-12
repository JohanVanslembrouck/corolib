/**
 * @file p1500-sync-3-parallel-rmis.cpp
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
        int ret2 = remoteObj2.op1(gin11, gin12, gout11, gout12);
        int ret3 = remoteObj3.op1(gin11, gin12, gout11, gout12);
        int result = ret1 + ret2 + ret3;
        printf("Class01::function1(): result = %d\n", result);
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01.function1(); });
    eventQueue.push([]() { class01.function1(); });
    eventQueue.run();
    return 0;
}
