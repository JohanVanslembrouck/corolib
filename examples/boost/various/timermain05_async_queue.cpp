/**
 * @file timermain05_async_queue.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer05_async_queue.h"

int main(int argc, char* argv[])
{
    set_priority(0x01);

    boost::asio::io_context ioContext;

    print(PRI1, "main: Timer05 c1(ioContext);\n");
    Timer05 c1(ioContext);

    print(PRI1, "main: async_task<void> t1 = c1.mainTasks();\n");
    async_task<void> t1 = c1.mainTasks();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: t1.wait();\n");
    t1.wait();

    print(PRI1, "main: return 0;\n");
    return 0;
}
