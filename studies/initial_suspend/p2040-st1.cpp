/**
 * @file p2040-st1.cpp
 * @brief Contains the main code that is included in
 * 
 * p2040e_void-st1-thread.cpp
 * p2040l_void-st1-thread.cpp
 * p2045e_void-st1-thread.cpp
 * p2045l_void-st1-thread.cpp
 *
 * and that will be compiled in 4 different ways by including different header files 
 * and using different constant definitions.
 *
 * @author Johan Vanslembrouck
 */

task foo() {
    mini_awaiter_ts ma;
    print(PRI1, "foo(): start_operation1(ma, delayafterstart = %d);\n", delayafterstart);
    start_operation1(ma, delayafterstart);
    print(PRI1, "foo(): int v = co_await ma;\n");
    int v = co_await ma;
    print(PRI1, "foo(): co_return %d;\n", v + 1);
    co_return v + 1;
}

task bar() {
    print(PRI1, "bar(): task f = foo();\n");
    task f = foo();
    print(PRI1, "bar(): int v = co_await f;\n");
    int v = co_await f;
    print(PRI1, "bar(): co_return %d;\n", v + 1);
    co_return v + 1;
}

int main() {
    set_print_level(0x01);

    print(PRI1, "main(): task b = bar();\n");
    task b = bar();
    print(PRI1, "main(): b.start();\n");
    b.start();

    // This delay gives the completion thread the time to run
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1100));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
