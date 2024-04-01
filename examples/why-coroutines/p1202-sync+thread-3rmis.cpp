/**
 * @file p1202-sync+thread-3rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <thread>

#include "common.h"

#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

int gval1 = 0;

class Class01
{
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1.op1(in1, in2, out1, out2);
        // 1 Do stuff
        if (ret1 == gval1) {
            int out3 = -1;
            int ret2 = remoteObj2.op2(in1, in2, out3);
            (void)ret2;
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = remoteObj3.op3(in1, out4, out5);
            (void)ret3;
            // 3 Do stuff
        }
    }
    void function2() { }
};

Class01 class01;

int main()
{
    printf("main();\n");
    std::thread th1(&Class01::function1, &class01, 11, 12); th1.join();
    std::thread th2(&Class01::function1, &class01, 21, 22); th2.join();
    return 0;
}
