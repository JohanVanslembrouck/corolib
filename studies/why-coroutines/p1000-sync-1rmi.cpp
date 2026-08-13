/**
 * @file p1000-sync-1rmi.cpp
 * @brief Example with 1 synchronous RMI.
 * This program is not reactive (responsive) because the RMI blocks the program
 * for the duration of the RMI.
 * 
 * @author Johan Vanslembrouck
 */

#include <stdio.h>

#include "common.h"
#include "p1000.h"

/**
 * @brief 
 * Class with a synchronous remote method invocation (RMI) in function1.
 *
 */
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

private:
    RemoteObject1 remoteObj1;
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
