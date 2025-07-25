/**
 * @file p1120_sn.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include "print.h"

#include "task_sn.h"
#include "class_async-thread.h"

int main()
{
    set_print_level(0x07);

    Class obj;
    print(PRI1, "main(): task a = obj.coroutine1();\n");
    task a = obj.coroutine1();

    print(PRI1, "main(): a.start();\n");
    a.start();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(2000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d = 0x%x;\n", v, v);
	
    print(PRI1, "main(): return 0;\n");
    return 0;
}
