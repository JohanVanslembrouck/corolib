/**
 * @file timermain06_async_semaphore.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer06_async_semaphore.h"

int main(int argc, char* argv[])
{
    set_priority(0x01);

    boost::asio::io_context ioContext;

    print(PRI1, "main: Timer06 c1(ioContext);\n");
    Timer06 c1(ioContext);

    print(PRI1, "main: async_task<void> t1 = c1.mainTask();\n");
    async_task<void> t1 = c1.mainTask();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: t1.wait();\n");
    t1.wait();

    print(PRI1, "main: return 0;\n");
    return 0;
}
