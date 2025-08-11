/**
 * @file p1515-async+thread-3-parallel-rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "p1200thr.h"

class Class01
{
public:
    int function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;
        int ret1 = -1;
        int ret2 = -1;
        int ret3 = -1;

        countingSemaphore cs;
        std::unique_lock<std::mutex> lock(cs.mu);

        remoteObj1thr.op1(in1, in2, out11, out12, ret1, cs);
        remoteObj1thr.op1(in1, in2, out21, out22, ret2, cs);
        remoteObj1thr.op1(in1, in2, out31, out32, ret3, cs);

        cs.cv.wait(lock, [&]() { return cs.done_count == 3; });

        int result = ret1 + ret2 + ret3;
        printf("Class01::function1(): result = %d\n", result);
        return result;
    }

private:
    RemoteObject1 remoteObj1;
    RemoteObject1 remoteObj2;
    RemoteObject1 remoteObj3;

    RemoteObject1Thr remoteObj1thr{ remoteObj1 };
    RemoteObject1Thr remoteObj2thr{ remoteObj2 };
    RemoteObject1Thr remoteObj3thr{ remoteObj2 };
};

int main()
{
    printf("main();\n");
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
    return 0;
}
