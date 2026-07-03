/**
 * @file p2050-op1x.cpp
 * @brief Contains the main code that is included in
 * 
 * p2050e_void-op1e-thread.cpp
 * p2050l_void-op1e-thread.cpp
 * p2055e_void-op1e-thread.cpp
 * p2055l_void-op1e-thread.cpp
 * 
 * p2060e_void-op1l-thread.cpp
 * p2060l_void-op1l-thread.cpp
 * p2065e_void-op1l-thread.cpp
 * p2065l_void-op1l-thread.cpp
 *
 * and that will be compiled in 8 different ways by including different header files 
 * and using different constant definitions.
 * 
 * @author Johan Vanslembrouck
 */

task foo() {
    print(PRI1, "foo(): auto op = start_operation1(delayafterstart = %d);\n", delayafterstart);
    auto op = start_operation1(delayafterstart);
    print(PRI1, "foo(): int v = co_await op;\n");
    int v = co_await op;
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
