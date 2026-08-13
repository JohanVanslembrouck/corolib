/**
 * @file p1500-sync-3-parallel-rmis.cpp
 * @brief Example with 3 consecutive synchronous RMIs.
 * This program is not reactive (responsive) because the RMI blocks the program
 * for the duration of the RMI.
 * Also, there is no parallelism.
 *
 * @author Johan Vanslembrouck
 */

#include <stdio.h>

#include "common.h"
#include "p1200.h"

class Class01
{
public:
    int function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;
  
        int ret1 = remoteObj1.op1(in1, in2, out11, out12);
        int ret2 = remoteObj2.op1(in1, in2, out21, out22);
        int ret3 = remoteObj3.op1(in1, in2, out31, out32);
        int result = ret1 + ret2 + ret3;
        printf("Class01::function1(): result = %d\n", result);
        return result;
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1 remoteObj2;
    RemoteObject1 remoteObj3;
};

int main()
{
    printf("main(): begin\n");
    Class01 class01;
    int ret1 = class01.function1(11, 12);
    int ret2 = class01.function1(21, 22);
    int ret3 = class01.function1(31, 32);
    int ret4 = class01.function1(41, 42);

    printf("\n");
    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    printf("main(): end\n");
    return 0;
}
