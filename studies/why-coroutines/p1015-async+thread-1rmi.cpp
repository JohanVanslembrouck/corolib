/**
 * @file p1015-async+thread-1rmi.cpp
 * @brief
 * Note how this implementation is very close to the one in p1000-sync-1rmi.cpp.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "common.h"
#include "p1000thr.h"

RemoteObject1 remoteObj1;
RemoteObject1Thr remoteObj1thr{remoteObj1};

class Class01
{
public:
    int function1(int in1, int in2)
    {
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1thr.op1(in1, in2, out1, out2);
        printf("Class01::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + in2 + out1 + out2 + ret1;
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    int ret1 = class01.function1(11, 12);
    int ret2 = class01.function1(21, 22);
    int ret3 = class01.function1(31, 32);
    int ret4 = class01.function1(41, 42);
    printf("\n");

    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}

