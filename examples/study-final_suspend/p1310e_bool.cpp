/**
 * @file p1310e_bool.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include "print.h"

#include "taske_bool.h"
#include "class_async.h"

mini_awaiter are1;

int main()
{
    set_print_level(0x03);        // Use 0x03 to follow the flow in corolib

    Class obj;
    print(PRI1, "main(): task a = obj.coroutine1();\n");
    task a = obj.coroutine1();
	
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();

    print("main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d;\n", v);
	
    print(PRI1, "main(): return 0;\n");
    return 0;
}