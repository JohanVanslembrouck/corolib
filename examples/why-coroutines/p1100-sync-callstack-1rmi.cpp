/**
 * @file p1100-sync-callstack-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1000.h"

RemoteObject1 remoteObj1;

class Layer01
{
public:
    int function1(int in1, int& out11, int& out12)
    {
        printf("Layer01::function1(): part 1\n");
        int ret1 = remoteObj1.op1(in1, in1, out11, out12);
        printf("Layer01::function1(): out11 = %d, out12 = %d, ret1 = %d\n", out11, out12, ret1);
        printf("Layer01::function1(): part 2\n");
        return ret1;
    }
};

Layer01 layer01;

class Layer02
{
public:
    int function1(int in1, int& out1)
    {
        printf("Layer02::function1(): part 1\n");
        int ret1 = layer01.function1(in1, out1, out2);
        printf("Layer02::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        printf("Layer02::function1(): part 2\n");
        return ret1;
    }
private:
    int    out2{0};
};

Layer02 layer02;

class Layer03
{
public:
    int function1(int in1)
    {
        printf("Layer03::function1(): part 1\n");
        int ret1 = layer02.function1(in1, out1);
        printf("Layer03::function1(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::function1(): part 2\n");
        return ret1;
    }
private:
    int    out1{0};
};

Layer03 layer03;

int main() {
    printf("main();\n");
    connect(event1, []() { layer03.function1(2); });
    connect(event2, []() { layer03.function1(3); });
    eventQueue.run();
    return 0;
}
