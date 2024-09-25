/**
 * @file p1500-sync-3-parallel-rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

class Class01
{
public:
    void function1(int in1, int in2)
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
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    class01.function1(11, 12);
    class01.function1(21, 22);
    return 0;
}
