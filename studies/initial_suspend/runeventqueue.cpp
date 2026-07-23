/**
 * @file runeventqueue.cpp
 * @brief This file contains the implementations of the runEventQueue functions.
 *
 * @author Johan Vanslembrouck
 */

#include "runeventqueue.h"

#include "print.h"

void runEventQueue(EventQueueThrFunctionVoidVoid& queue)
{
    while (!queue.isEmpty())
    {
        print(PRI2, "runEventQueue(): FunctionVoidVoid op = queue.pop();\n");
        FunctionVoidVoid op = queue.pop();
        print(PRI2, "runEventQueue(): op();\n");
        op();
    }
}

void runEventQueue(EventQueueThrFunctionVoidBool& queue)
{
    while (!queue.isEmpty())
    {
        print(PRI2, "runEventQueue(): FunctionVoidBool op = queue.pop();\n");
        FunctionVoidBool op = queue.pop();
        // In a real implementation, the value will be calculated from information
        // available on a lower level.
        // In this simulation, this value is hard-coded to true.
        print(PRI2, "runEventQueue(): op(true);\n");
        op(true);
    }
}

void runEventQueue(EventQueueThrFunctionVoidInt& queue)
{
    while (!queue.isEmpty())
    {
        print(PRI2, "runEventQueue(): FunctionVoidInt op = queue.pop();\n");
        FunctionVoidInt op = queue.pop();
        // In a real implementation, the value will be calculated from information
        // available on a lower level.
        // In this simulation, this value is hard-coded to 0.
        print(PRI2, "runEventQueue(): op(0);\n");
        op(0);
    }
}

void runEventQueue(EventQueueThrFunctionVoidString& queue)
{
    while (!queue.isEmpty())
    {
        print(PRI2, "runEventQueue(): FunctionVoidString op = queue.pop();\n");
        FunctionVoidString op = queue.pop();
        // In a real implementation, the value will be calculated from information
        // available on a lower level.
        // In this simulation, this value is hard-coded to "Default output string".
        print(PRI2, "runEventQueue(): op(\"Default output string\");\n");
        op("Default output string");
    }
}

void runEventQueue(EventQueueThrFunctionX& queue)
{
    while (!queue.isEmpty())
    {
        print(PRI2, "runEventQueue(): FunctionVoidX op = queue.pop();\n");
        FunctionVoidX op = queue.pop();

        if (std::holds_alternative<FunctionVoidBool>(op)) {
            FunctionVoidBool op2 = std::get<FunctionVoidBool>(op);
            print(PRI2, "runEventQueue(): op(true);\n");
            op2(true);
        }
        else if (std::holds_alternative<FunctionVoidInt>(op)) {
            FunctionVoidInt op2 = std::get<FunctionVoidInt>(op);
            print(PRI2, "runEventQueue(): op(0);\n");
            op2(0);
        }
        else if (std::holds_alternative<FunctionVoidString>(op)) {
            FunctionVoidString op2 = std::get<FunctionVoidString>(op);
            print(PRI2, "runEventQueue(): op2(\"Default output string\"); \n");
            op2("Default output string");
        }
        else if (std::holds_alternative<FunctionVoidVoid>(op)) {
            FunctionVoidVoid op2 = std::get<FunctionVoidVoid>(op);
            print(PRI2, "runEventQueue(): op2(); \n");
            op2();
        }
    }
}
