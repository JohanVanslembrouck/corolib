/**
 * @file p2000-sync-3rmis.cpp
 * @brief
 * This example simulates 3 synchronous remote method invocations (RMIs). 
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "p2000-common.h"

class Class1
{
public:
    int operationA(int in1, int& out1)
    {
        printf("client part 1: Prepare input for operation1\n");
        int in11 = in1;
        int in12 = 2 + in1;
        int out11 = 0;
        printf("Calling operation1(in11 = %d, int12 = %d, out11 = %d) => %d\n", in11, in12, out11, 0);
        int ret1 = operation1(in11, in12, out11);

        printf("client part 2: Process output from operation1 and prepare input for operation2\n");
        int in21 = in11;
        int out21 = 0;
        int out22 = 0;
        printf("Calling operation2(in21 = %d, out21 = %d, out22 = %d) => %d\n", in21, out21, out22, 0);
        int ret2 = operation2(in21, out21, out22);

        printf("client part 3: Process output from operation2 and prepare input for operation3\n");
        int in31 = out21;
        int inout31 = out22;
        int out31 = 0;
        printf("Calling operation3(in31 = %d, inout31 = %d, out31 = %d) => %d\n", in31, inout31, out31, 0);
        int ret3 = operation3(in31, inout31, out31);
        
        printf("client part 4: Process output from operation3\n"); 
        out1 = inout31 + out31;
        int res = ret1 + ret2 + ret3;
        return res;
    }

    int operation1(int in11, int in12, int& out11)
    {
        int ret;
        operation1_server(in11, in12, out11, ret);
        printf("Returning from operation1(in11 = %d, int12 = %d, out11 = %d) => %d\n", in11, in12, out11, ret);
        return ret;
    }

    int operation2(int in21, int& out21, int& out22)
    {
        int ret;
        operation2_server(in21, out21, out22, ret);
        printf("Returning from operation2(in21 = %d, out21 = %d, out22 = %d) => %d\n", in21, out21, out22, ret);
        return ret;
    }

    int operation3(int in31, int& inout31, int& out31)
    {
        int ret;
        operation3_server(in31, inout31, out31, ret);
        printf("Returning from operation3(in31 = %d, inout31 = %d, out31 = %d) => %d\n", in31, inout31, out31, ret);
        return ret;
    }
};

int main()
{
    int out1 = 0;
    Class1 obj1;
    int ret1 = obj1.operationA(10, out1);
    printf("out1 = %d, ret = %d\n", out1, ret1);
    int ret2 = obj1.operationA(20, out1);
    printf("out1 = %d, ret = %d\n", out1, ret2);
    return 0;
}
