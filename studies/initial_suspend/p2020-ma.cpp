/**
 * @file p2020-ma.cpp
 * @brief Contains the main code that is included in
 * 
 * p2020e_void-ma.cpp
 * p2020l_void-ma.cpp
 *
 * and that will be compiled in 2 different ways by including different header files.
 *
 * @author Johan Vanslembrouck
 */

mini_awaiter ma;

task foo() {
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
    print(PRI1, "main(): ma.set_result_and_resume(10);\n");
    ma.set_result_and_resume(10);

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
