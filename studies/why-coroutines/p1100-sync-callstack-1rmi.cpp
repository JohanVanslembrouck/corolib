/**
 * @file p1100-sync-callstack-1rmi.cpp
 * @brief Implementation of a callstack with 3 application layers (Layer03, Layer02, Layer01).
 * The application calls layer03.function1, which calls layer2.function2, which calls layer1.function1,
 * which calls remoteOb1.op.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
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
    int function1(int in1, int& out1, int& out2)
    {
        printf("Layer01::function1(in1 = %d, out1 = %d, out2 = %d)\n", in1, out1, out2);
        int ret1 = remoteObj1.op1(in1, in1, out1, out2);
        printf("Layer01::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + ret1;
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
        printf("Layer02::function1(in1 = %d, out1 = %d)\n", in1, out1);
        int out2 = -1;
        int ret1 = layer01.function1(in1, out1, out2);
        printf("Layer02::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + out2 + ret1;
    }
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
        printf("Layer03::function1(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = layer02.function1(in1, out1);
        printf("Layer03::function1(): out1 = %d, ret1 = %d\n", out1, ret1);
        return in1 + out1 + ret1;
    }
};

Layer03 layer03;

int main() {
    printf("main();\n");
    int ret1 = layer03.function1(2);
    int ret2 = layer03.function1(3);

    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
