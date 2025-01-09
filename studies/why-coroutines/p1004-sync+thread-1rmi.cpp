/**
 * @file p1004-sync+thread-1rmi.cpp
 * @brief This is a variant of p1002-sync+thread-1rmi.cpp.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <future>

#include "common.h"
#include "p1000.h"

RemoteObject1 remoteObj1;

/**
 * @brief Class with a simple remote method invocation (RMI) in function1.
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
};

Class01 class01;

int main()
{
    printf("main();\n");
    std::future<int> t1 = std::async(std::launch::async, []() { return class01.function1(11, 12); });
    std::future<int> t2 = std::async(std::launch::async, []() { return class01.function1(21, 22); });
    printf("\n");

    int ret1 = t1.get();
    printf("main(): ret1 = %d\n", ret1);
    int ret2 = t2.get();
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
