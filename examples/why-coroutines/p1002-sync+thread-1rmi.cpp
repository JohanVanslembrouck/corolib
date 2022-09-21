/**
 * @file p1002-sync+thread-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <thread>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1000.h"

RemoteObject1 remoteObj1;

class Class01
{
public:
    int function1()
	{
        printf("Class01::function1(): part 1\n");
        int ret1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
        printf("Class01::function1(): gout11 = %d, gout12 = %d, ret1 = %d\n", gout11, gout12, ret1);
        printf("Class01::function1(): part 2\n");
        return ret1;
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { std::thread th(&Class01::function1, &class01); th.join(); });
    eventQueue.push([]() { std::thread th(&Class01::function1, &class01); th.join(); });
    eventQueue.run();
    return 0;
}
