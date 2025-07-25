/**
 * @file p1100e_sn.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include "print.h"

#include "taske_sn.h"
#include "class_sync.h"

int main()
{
    set_print_level(0x07);

    Class obj;
    print(PRI1, "main(): task a = obj.coroutine1();\n");
    task a = obj.coroutine1();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d = 0x%x;\n", v, v);
	
    print(PRI1, "main(): return 0;\n");
    return 0;
}
