/**
 * @file p1100-sync-callstack-1rmi.cpp
 * @brief Implementation of a callstack with 3 application layers (Layer03, Layer02, Layer01).
 * The application calls layer03.function1, which calls layer2.function2, which calls layer1.function1,
 * which calls remoteOb1.op.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1000.h"

RemoteObject1 remoteObj1;

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1
 * Upper layer: Layer02 (but not known by Layer01)
 *
 */
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

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (but not known by Layer02)
 *
 */
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

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (but not known by Layer03)
 *
 */
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

    int function2(int in1)
    {
        printf("Layer03::function2(): part 1\n");
        int ret1 = layer02.function1(in1, out1);
        printf("Layer03::function2(): out1 = %d, ret1 = %d\n", out1, ret1);
        printf("Layer03::function2(): part 2\n");
        return ret1;
    }
private:
    int    out1{0};
};

Layer03 layer03;

EventQueue eventQueue;

int main() {
    printf("main();\n");
    layer03.function1(2);
    layer03.function2(3);
    eventQueue.run();
    return 0;
}
