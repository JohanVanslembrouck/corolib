/**
 * @file p1510-async-3-parallel-rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

class Class01
{
public:
    void function1()
    {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        remoteObj2.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { this->function1b(out1, out2, ret1); });
        remoteObj3.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { this->function1c(out1, out2, ret1); });
    }

    void function1a(int out11, int out12, int ret1)
    {
        printf("Class01::function1a(%d, %d, %d)\n", out11, out12, ret1);
        call1finished = true;
        result1 = ret1;
        function1d();
    }
    
    void function1b(int out11, int out12, int ret1)
    {
        printf("Class01::function1b(%d, %d, %d)\n", out11, out12, ret1);
        call2finished = true;
        result2 = ret1;
        function1d();
    }
    
    void function1c(int out11, int out12, int ret1)
    {
        printf("Class01::function1c(%d, %d, %d)\n", out11, out12, ret1);
        call3finished = true;
        result3 = ret1;
        function1d();
    }

    void function1d()
    {
		if (call1finished && call2finished && call3finished)
			printf("Class01::function1d: result = %d\n", result1 + result2 + result3);
    }
    
private:
    bool call1finished{false};
    bool call2finished{false};
    bool call3finished{false};
    int result1{0};
    int result2{0};
    int result3{0};
};

Class01 class01;

int main()
{
    printf("main();\n");
    eventQueue.push([]() { class01.function1(); });
    eventQueue.push([]() { class01.function1(); });
    eventQueue.run();
    return 0;
}
