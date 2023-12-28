/**
 * @file p1010-async-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1000.h"

RemoteObject1 remoteObj1;

/**
 * @brief Asynchronous version of Class01 in p1000-sync-1rmi.cpp
 *
 */
class Class01
{
public:
    
    /**
     * @brief
     *
     */
    void function1()
    {
        printf("Class01::function1(): part 1\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1)
            { 
                this->function1_cb(out1, out2, ret1);
            });
    }

    /**
     * @brief callback function with the out parameters and the return value
     *
     */
    void function1_cb(int out11, int out12, int ret1)
    {
        printf("Class01::function1_cb(out11 = %d, out12 = %d, ret1 = %d)\n", out11, out12, ret1);
        printf("Class01::function1_cb(): part 2\n");
    }
    
    /**
     * @brief alternative version of function1 that avoids the introduction
     * of a callback function by placing part 2 of the original synchronous function
     * in the body of the lambda.
     *
     */
    void function1alt()
    {
        printf("Class01::function1alt(): part 1\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1)
            { 
                printf("Class01::function1alt(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
                printf("Class01::function1alt(): part 2\n");
            });
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01.function1();
    class01.function1alt();
    eventQueue.run();
    return 0;
}
