/**
 * @file p1050-sync-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "p1050.h"                              // difference with p1000-sync-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1000-sync-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1000-sync-1rmi.cpp

class Class01
{
public:
    int function1(int in1, int in2)
	{
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1.op1(in1, in2, out1, out2);
        printf("Class01::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + in2 + out1 + out2 + ret1;
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    int ret1 = class01.function1(11, 12);
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = class01.function1(21, 22);
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
