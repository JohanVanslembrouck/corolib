/**
 * @file p1200-sync-3rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (ohan.vanslembrouck@gmail.com)
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
    void function1(int in1, int in2, int testval)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1.op1(in1, in2, out1, out2);
        printf("Class01::function: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1 Do stuff
        if (ret1 == testval) {
            int out3 = -1;
            int ret2 = remoteObj2.op2(in1, in2, out3);
            printf("Class01::function: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = remoteObj3.op3(in1, out4, out5);
            printf("Class01::function: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3 Do stuff
        }
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    class01.function1(11, 12, 10);
    printf("\n");
    class01.function1(11, 12, 23);
    return 0;
}
