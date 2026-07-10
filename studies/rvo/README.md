# On the use of (named) return value optimization

The test applications in this directory investigate whether various compilers use (named) return value optimization
in a reliable and consistent way, and if not, how we can deal with this.

## Definition

From https://en.wikipedia.org/wiki/Copy_elision:

In the context of the C++ programming language, return value optimization (RVO) is a compiler optimization 
that involves eliminating the temporary object created to hold a function's return value.

The following code shows two functions to which
return value optimization (RVO) and named return value optimization (NVRO)
is applicable.

```c++
async_operation function1() {
    // other code
    return async_operation;                 // return value optimization
}

async_operation function2() {    
    async_operation ret;
    // other code, possibly using ret
    return ret;                              // named return value optimization
}

void test() {
    async_operation op1 = function1();
    async_operation op2 = function2();
    // code using op1 and op2
}
```

## rvo1.cpp: investigating the use of RVO

Consider the following definition of an operation that will be used in the examples below.
The definition of async_operation can be found in rvo1.cpp, but it is not important in this context.

```c++
void start_operation_impl(int i, async_operation* ao) {
    printf("%p = ao: start_operation_impl\n", ao);
    async_op(i, [ao](int v) {
        if (ao)
            ao->set_value(v);
        });
}

async_operation start_operation(int i) {
    async_operation ret;
    start_operation_impl(i, &ret);
    printf("%p = &ret: start_operation\n", &ret);
    return ret;
}
```

Function start_operation() creates a local async_operation object, ret,
passes its address to start_operation_impl(), and then returns ret.

Function async_op() is defined as follows:

```c++
void async_op(int i, std::function<void(int)>&& op) {
    if (i <= 10)
        op(i * i);
    else
        eventHandler.set(std::move(op), i * i);
}
```

If async_op is passed a number <= 10, it calculates the square of the number itself.
Otherwise, it will contact a remote application (bypassed in this implementation) that will do the calculation.
async_op is also passed a lambda function that will be called from the event loop when the remote application
has sent the response.

### test00: pass object as argument

```c++
void test00() {
    printf("\n--- test00: pass object as argument ---\n");
    async_operation ao;
    start_operation_impl(20, &ao);
    eventHandler.run();
    int res = ao.get_value();
    printf("test00: res = %d\n", res);
    if (res != 400)
        printf("test04: Expected 400, received res = %d !!!\n", ao.get_value());
}}
```

Function test00() calls start_operation_impl() and passes the address of a local async_operation object, ao.
We run the event handler (eventHandler.run()), which will call the lambda function passed to it from start_operation_impl().
After that, we can obtain the result from the async_operation object by calling the member function get_value().

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test00: pass object as argument ---
0000006ED70FF808: async_operation::async_operation(): m_value = -1
0000006ED70FF808 = ao: start_operation_impl
EventHandler::set(..., 400)
EventHandler::run()
0000006ED70FF808: async_operation::set_value(400)
0000006ED70FF808: async_operation::get_value() returns 400
test00: res = 400
0000006ED70FF808: async_operation::~async_operation(): m_value = -2
```

The output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe)
is the same (apart from object addresses that usually change with every run).

The output of both applications is correct.

However, the coding style is a bit clumsy, by first having to declare a local object
and then passing its address (or a reference it) to start_operation_impl(). Can we do better?

### test01: test RVO

Function test01() uses a more natural coding style:

```c++
void test01() {
    printf("\n--- test01: test RVO ---\n");
    async_operation ao = start_operation(20);
    eventHandler.run();
    int res = ao.get_value();
    printf("test01: res = %d\n", res);
    if (res != 400)
        printf("test01: Expected 400, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test01: test RVO ---
000000DEEA58FC88: async_operation::async_operation(): m_value = -1
000000DEEA58FC88 = ao: start_operation_impl
EventHandler::set(..., 400)
000000DEEA58FC88 = &ret: start_operation
000000DEEA58FCE8: async_operation::async_operation(async_operation&&): m_value = -1
000000DEEA58FC88: async_operation::~async_operation(): m_value = -2
EventHandler::run()
000000DEEA58FC88: async_operation::set_value(400): async_operation has gone out of scope: m_value = -956557928
000000DEEA58FCE8: async_operation::get_value() returns -1
test01: res = -1
000000DEEA58FCE8: async_operation::get_value() returns -1
test01: Expected 400, received res = -1 !!!
000000DEEA58FCE8: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with  MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test01: test RVO ---
000000B7E79BFAE8: async_operation::async_operation(): m_value = -1
000000B7E79BFAE8 = ao: start_operation_impl
EventHandler::set(..., 400)
000000B7E79BFAE8 = &ret: start_operation
EventHandler::run()
000000B7E79BFAE8: async_operation::set_value(400)
000000B7E79BFAE8: async_operation::get_value() returns 400
test01: res = 400
000000B7E79BFAE8: async_operation::~async_operation(): m_value = -2
```

The output of rvo1a.exe is wrong (it still contains the initial value -1 assigned by the constructor),
the output of rvo1b.exe is correct.

The second output shows that the compiler has used NRVO, while in the first compilation,
the return from start_operation() to test01()
creates a new object using the move constructor, after which the original object goes out of scope.

The lambda function only knows the address of the original object,
which has gone out of scope at the moment
the lambda function is called from the event handler.

Before looking for a solution, let's try a few other test functions.

### test02: test RVO: immediate completion

Function test02() is implemented as follows:

```c++
void test02() {
    printf("\n--- test02: test RVO: immediate completion ---\n");
    async_operation ao = start_operation(10);
    eventHandler.run();
    int res = ao.get_value();
    printf("test02: res = %d\n", res);
    if (res != 100)
        printf("test02: Expected 400, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test02: test RVO: immediate completion ---
000000DEEA58FC88: async_operation::async_operation(): m_value = -1
000000DEEA58FC88 = ao: start_operation_impl
000000DEEA58FC88: async_operation::set_value(100)
000000DEEA58FC88 = &ret: start_operation
000000DEEA58FCE8: async_operation::async_operation(async_operation&&): m_value = 100
000000DEEA58FC88: async_operation::~async_operation(): m_value = -2
EventHandler::run()
000000DEEA58FCE8: async_operation::get_value() returns 100
test02: res = 100
000000DEEA58FCE8: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test02: test RVO: immediate completion ---
000000B7E79BFAE8: async_operation::async_operation(): m_value = -1
000000B7E79BFAE8 = ao: start_operation_impl
000000B7E79BFAE8: async_operation::set_value(100)
000000B7E79BFAE8 = &ret: start_operation
EventHandler::run()
000000B7E79BFAE8: async_operation::get_value() returns 100
test02: res = 100
000000B7E79BFAE8: async_operation::~async_operation(): m_value = -2
```

The output of both applications is correct.
Although rvo1a.exe uses an intermediate object,
this object already contains the correct result at the moment it is "moved" to the final object in test02().

### test03: object goes out of scope

Function test03() is implemented as follows:

```c++
void test03a() {
    async_operation ao = start_operation(20);
}

void test03() {
    printf("\n--- test03: object goes out of scope ---\n");
    test03a();
    eventHandler.run();
}

```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test03: object goes out of scope ---
000000DEEA58FC58: async_operation::async_operation(): m_value = -1
000000DEEA58FC58 = ao: start_operation_impl
EventHandler::set(..., 400)
000000DEEA58FC58 = &ret: start_operation
000000DEEA58FCB8: async_operation::async_operation(async_operation&&): m_value = -1
000000DEEA58FC58: async_operation::~async_operation(): m_value = -2
000000DEEA58FCB8: async_operation::~async_operation(): m_value = -2
EventHandler::run()
000000DEEA58FC58: async_operation::set_value(400): async_operation has gone out of scope: m_value = 0
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test03: object goes out of scope ---
000000B7E79BFAB8: async_operation::async_operation(): m_value = -1
000000B7E79BFAB8 = ao: start_operation_impl
EventHandler::set(..., 400)
000000B7E79BFAB8 = &ret: start_operation
000000B7E79BFAB8: async_operation::~async_operation(): m_value = -2
EventHandler::run()
000000B7E79BFAB8: async_operation::set_value(400): async_operation has gone out of scope: m_value = -4
```

As both traces show, the async_operation object(s) has(have) gone out of scope at the moment
the event handler runs: the lambda function writes the result to an object whose destructor has been called.

Because the async_operation object(s) are located on the stack, this does not lead to a segmentation fault,
but it can lead to faulty behavior if its memory has been taken by another object (or objects), whose value
will be overwritten.

### test04: auxiliary function returns async_operation

Function test04() is implemented as follows:

```c++
async_operation test04a() {
    async_operation ao = start_operation(20);
    printf("test04a: &ao = %p\n", &ao);
    return ao;
}

void test04() {
    printf("\n--- test04: auxiliary function returns async_operation ---\n");
    async_operation ao = test04a();
    eventHandler.run();
    int res = ao.get_value();
    printf("test04: res = %d\n", res);
    if (res != 400)
        printf("test04: Expected 400, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test04: auxiliary function returns async_operation ---
000000DEEA58FC28: async_operation::async_operation(): m_value = -1
000000DEEA58FC28 = ao: start_operation_impl
EventHandler::set(..., 400)
000000DEEA58FC28 = &ret: start_operation
000000DEEA58FC88: async_operation::async_operation(async_operation&&): m_value = -1
000000DEEA58FC28: async_operation::~async_operation(): m_value = -2
test04a: &ao = 000000DEEA58FC88
000000DEEA58FCE8: async_operation::async_operation(async_operation&&): m_value = -1
000000DEEA58FC88: async_operation::~async_operation(): m_value = -2
EventHandler::run()
000000DEEA58FC28: async_operation::set_value(400): async_operation has gone out of scope: m_value = 0
000000DEEA58FCE8: async_operation::get_value() returns -1
test04: res = -1
000000DEEA58FCE8: async_operation::get_value() returns -1
test04: Expected 400, received res = -1 !!!
000000DEEA58FCE8: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test04: auxiliary function returns async_operation ---
000000B7E79BFAE8: async_operation::async_operation(): m_value = -1
000000B7E79BFAE8 = ao: start_operation_impl
EventHandler::set(..., 400)
000000B7E79BFAE8 = &ret: start_operation
test04a: &ao = 000000B7E79BFAE8
EventHandler::run()
000000B7E79BFAE8: async_operation::set_value(400)
000000B7E79BFAE8: async_operation::get_value() returns 400
test04: res = 400
000000B7E79BFAE8: async_operation::~async_operation(): m_value = -2
```

The output of rvo1a.exe is wrong (it still contains the initial value -1 assigned by the constructor),
the output of rvo1b.exe is correct.

The second output shows that the compiler has used RVO twice, while in the first compilation,
the move constructor is used twice, after which the original objects go out of scope.

## rvo2.cpp: avoid using objects that have gone out of scope

The solution has to resolve all problems in rvo1.cpp.

Consider the following modified definitions of an operation:

```c++
void start_operation_impl(int i, int index) {
    async_op(i, [index](int i) {
        async_operation* ao = async_ops.get_entry(index);
        printf("start_operation_impl: index = %d, ao = %p\n", index, ao);
        if (ao)
            ao->set_value(i);
        });
}

async_operation start_operation(int i) {
    int idx = async_ops.get_free_index();
    async_operation ret(idx);
    printf("%p = &ret: start_operation, idx = %d\n", &ret, idx);
    start_operation_impl(i, idx);
    return ret;
}
```

The object async_ops can hold the addresses of a number of async_operation objects, e.g. using an array or a vector.
Its member function get_free_index() finds a free entry in this array (vector).
In its constructor, an async_operation object writes its address into the array at the index passed to it.
In the move constructor, the new object will overwrite the address of the original object with its own address.
The destructor of an async_operation object will normally remove its address from the array,
if it still contains a valid index.
However, in the move constructor, the original object will be denied access to the array by setting its index to -1.
This way, it will not be able anymore to clear its address when it goes out of scope and its destructor is called.

All test functions have the same implementation as in rvo1.cpp, except test00 that is empty.

### test00: pass object as argument

Empty implementation. Not applicable.

### test01: test RVO

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test01: test RVO ---
000000B9C62FF678: async_operation::async_operation(): m_value = -1
000000B9C62FF678 = &ret: start_operation, idx = 0
EventHandler::set(..., 400)
000000B9C62FF6D8: async_operation::async_operation(async_operation&&): m_value = -1
000000B9C62FF678: async_operation::~async_operation(): m_value = -1
EventHandler::run()
start_operation_impl: index = 0, ao = 000000B9C62FF6D8
000000B9C62FF6D8: async_operation::set_value(400)
000000B9C62FF6D8: async_operation::get_value() returns 400
test01: res = 400
000000B9C62FF6D8: async_operation::~async_operation(): m_value = 400
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test01: test RVO ---
000000ABAAD3FA68: async_operation::async_operation(): m_value = -1
000000ABAAD3FA68 = &ret: start_operation, idx = 0
EventHandler::set(..., 400)
EventHandler::run()
start_operation_impl: index = 0, ao = 000000ABAAD3FA68
000000ABAAD3FA68: async_operation::set_value(400)
000000ABAAD3FA68: async_operation::get_value() returns 400
test01: res = 400
000000ABAAD3FA68: async_operation::~async_operation(): m_value = 400
```

The output of both is correct.

### test02: test RVO: immediate completion

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test02: test RVO: immediate completion ---
000000B9C62FF678: async_operation::async_operation(): m_value = -1
000000B9C62FF678 = &ret: start_operation, idx = 1
start_operation_impl: index = 1, ao = 000000B9C62FF678
000000B9C62FF678: async_operation::set_value(100)
000000B9C62FF6D8: async_operation::async_operation(async_operation&&): m_value = 100
000000B9C62FF678: async_operation::~async_operation(): m_value = -1
EventHandler::run()
000000B9C62FF6D8: async_operation::get_value() returns 100
test02: res = 100
000000B9C62FF6D8: async_operation::~async_operation(): m_value = 100
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test02: test RVO: immediate completion ---
000000B9C62FF678: async_operation::async_operation(): m_value = -1
000000B9C62FF678 = &ret: start_operation, idx = 1
start_operation_impl: index = 1, ao = 000000B9C62FF678
000000B9C62FF678: async_operation::set_value(100)
000000B9C62FF6D8: async_operation::async_operation(async_operation&&): m_value = 100
000000B9C62FF678: async_operation::~async_operation(): m_value = -1
EventHandler::run()
000000B9C62FF6D8: async_operation::get_value() returns 100
test02: res = 100
000000B9C62FF6D8: async_operation::~async_operation(): m_value = 100
```

The output of both is correct.

### test03: object goes out of scope

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test03: object goes out of scope ---
000000B9C62FF648: async_operation::async_operation(): m_value = -1
000000B9C62FF648 = &ret: start_operation, idx = 2
EventHandler::set(..., 400)
000000B9C62FF6A8: async_operation::async_operation(async_operation&&): m_value = -1
000000B9C62FF648: async_operation::~async_operation(): m_value = -1
000000B9C62FF6A8: async_operation::~async_operation(): m_value = -1
EventHandler::run()
start_operation_impl: index = 2, ao = 0000000000000000
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test03: object goes out of scope ---
000000ABAAD3FA38: async_operation::async_operation(): m_value = -1
000000ABAAD3FA38 = &ret: start_operation, idx = 2
EventHandler::set(..., 400)
000000ABAAD3FA38: async_operation::~async_operation(): m_value = -1
EventHandler::run()
start_operation_impl: index = 2, ao = 0000000000000000
```

In both cases, the lambda function will not attempt to write to an async_operation object,
because its address has been cleared from the array at the index passed to the lambda function.

This assumes, of course, that the entry has not yet been taken by another object.
This can be assured by making the array large enough, or better,
by using a timestamp that is passed to the lambda function at the moment the original object is created.

### test04: auxiliary function returns async_operation

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test04: auxiliary function returns async_operation ---
000000B9C62FF618: async_operation::async_operation(): m_value = -1
000000B9C62FF618 = &ret: start_operation, idx = 3
EventHandler::set(..., 400)
000000B9C62FF678: async_operation::async_operation(async_operation&&): m_value = -1
000000B9C62FF618: async_operation::~async_operation(): m_value = -1
test04a: &ao = 000000B9C62FF678
000000B9C62FF6D8: async_operation::async_operation(async_operation&&): m_value = -1
000000B9C62FF678: async_operation::~async_operation(): m_value = -1
EventHandler::run()
start_operation_impl: index = 3, ao = 000000B9C62FF6D8
000000B9C62FF6D8: async_operation::set_value(400)
000000B9C62FF6D8: async_operation::get_value() returns 400
test04: res = 400
000000B9C62FF6D8: async_operation::~async_operation(): m_value = 400
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test04: auxiliary function returns async_operation ---
000000ABAAD3FA68: async_operation::async_operation(): m_value = -1
000000ABAAD3FA68 = &ret: start_operation, idx = 3
EventHandler::set(..., 400)
test04a: &ao = 000000ABAAD3FA68
EventHandler::run()
start_operation_impl: index = 3, ao = 000000ABAAD3FA68
000000ABAAD3FA68: async_operation::set_value(400)
000000ABAAD3FA68: async_operation::get_value() returns 400
test04: res = 400
000000ABAAD3FA68: async_operation::~async_operation(): m_value = 400
```

The output of both is correct.

## rvo3.cpp

In this implementation, function start_operation has been "promoted" to a class
with start_operation_impl being a protected function.

```c++
class start_operation : public async_operation
{
public:
    start_operation(int i)
    {
        printf("start_operation::start_operation(%d)\n", i);
        start_operation_impl(i);
    }

    int get_value() {
        printf("start_operation::get_value()\n");
        return async_operation::get_value();
    }

protected:
    void start_operation_impl(int i) {
        printf("start_operation::start_operation_impl(%d)\n", i);
        async_op(i, [this](int v) {
            set_value(v);
            });
    }
};
```

### test00: pass object as argument

Empty implementation. Not applicable.

### test01: test RVO

Function test01() is implemented as follows:

```c++
void test01() {
    printf("\n--- test01: ---\n");
    start_operation ao(20);
    eventHandler.run();
    int res = ao.get_value();
    printf("test01: res = %d\n", res);
    if (res != 400)
        printf("test01: Expected 400, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo3a.exe):

```
--- test01: ---
00000049A9BCFD78: async_operation::async_operation(): m_value = -1
00000049A9BCFD78: start_operation::start_operation(20)
00000049A9BCFD78: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
EventHandler::run()
00000049A9BCFD78: async_operation::set_value(400)
00000049A9BCFD78: start_operation::get_value()
00000049A9BCFD78: async_operation::get_value() returns 400
test01: res = 400
00000049A9BCFD78: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo3b.exe):

```
--- test01: ---
0000008EE8CFF888: async_operation::async_operation(): m_value = -1
0000008EE8CFF888: start_operation::start_operation(20)
0000008EE8CFF888: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
EventHandler::run()
0000008EE8CFF888: async_operation::set_value(400)
0000008EE8CFF888: start_operation::get_value()
0000008EE8CFF888: async_operation::get_value() returns 400
test01: res = 400
0000008EE8CFF888: async_operation::~async_operation(): m_value = -2
```

The output of both is correct.

### test02: test RVO: immediate completion

Function test02() is implemented as follows:

```c++
void test02() {
    printf("\n--- test02: immediate completion ---\n");
    start_operation ao(10);
    eventHandler.run();
    int res = ao.get_value();
    printf("test02: res = %d\n", res);
    if (res != 100)
        printf("test04: Expected 100, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo3a.exe):

```
--- test02: immediate completion ---
00000049A9BCFD78: async_operation::async_operation(): m_value = -1
00000049A9BCFD78: start_operation::start_operation(10)
00000049A9BCFD78: start_operation::start_operation_impl(10)
00000049A9BCFD78: async_operation::set_value(100)
EventHandler::run()
00000049A9BCFD78: start_operation::get_value()
00000049A9BCFD78: async_operation::get_value() returns 100
test02: res = 100
00000049A9BCFD78: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo3b.exe):

```
--- test02: immediate completion ---
0000008EE8CFF888: async_operation::async_operation(): m_value = -1
0000008EE8CFF888: start_operation::start_operation(10)
0000008EE8CFF888: start_operation::start_operation_impl(10)
0000008EE8CFF888: async_operation::set_value(100)
EventHandler::run()
0000008EE8CFF888: start_operation::get_value()
0000008EE8CFF888: async_operation::get_value() returns 100
test02: res = 100
0000008EE8CFF888: async_operation::~async_operation(): m_value = -2
```

The output of both is correct.

### test03: object goes out of scope

Function test03() is implemented as follows:

```c++
void test03a() {
    start_operation ao(20);
    printf("test03a: &ao = %p\n", &ao);
}

void test03() {
    printf("\n--- test03: object goes out of scope ---\n");
    test03a();
    eventHandler.run();
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo3a.exe):

```
--- test03: object goes out of scope ---
00000049A9BCFD48: async_operation::async_operation(): m_value = -1
00000049A9BCFD48: start_operation::start_operation(20)
00000049A9BCFD48: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
test03a: &ao = 00000049A9BCFD48
00000049A9BCFD48: async_operation::~async_operation(): m_value = -2
EventHandler::run()
00000049A9BCFD48: async_operation::set_value(400): async_operation has gone out of scope: m_value = 1106318744
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo3b.exe):

```
--- test03: object goes out of scope ---
0000008EE8CFF858: async_operation::async_operation(): m_value = -1
0000008EE8CFF858: start_operation::start_operation(20)
0000008EE8CFF858: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
test03a: &ao = 0000008EE8CFF858
0000008EE8CFF858: async_operation::~async_operation(): m_value = -2
EventHandler::run()
0000008EE8CFF858: async_operation::set_value(400): async_operation has gone out of scope: m_value = -4
```

In both cases, function set_value() detects that the object has gone out of scope and it will not attempt to write to it.

### test04: auxiliary function returns async_operation

Function test04() is implemented as follows:

```c++
start_operation test04a() {
    start_operation ao(20);
    return ao;
}

void test04() {
    printf("\n--- test04: auxiliary function returns operation ---\n");
    start_operation ao = test04a();
    eventHandler.run();
    int res = ao.get_value();
    printf("test04: res = %d\n", res);
    if (res != 400)
        printf("test04: Expected 400, received res = %d !!!\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo3a.exe):

```
--- test04: auxiliary function returns operation ---
00000049A9BCFD18: async_operation::async_operation(): m_value = -1
00000049A9BCFD18: start_operation::start_operation(20)
00000049A9BCFD18: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
00000049A9BCFD78: async_operation::async_operation(async_operation&&): m_value = -1
00000049A9BCFD18: async_operation::~async_operation(): m_value = -2
EventHandler::run()
00000049A9BCFD18: async_operation::set_value(400): async_operation has gone out of scope: m_value = 1106318744
00000049A9BCFD78: start_operation::get_value()
00000049A9BCFD78: async_operation::get_value() returns -1
test04: res = -1
00000049A9BCFD78: start_operation::get_value()
00000049A9BCFD78: async_operation::get_value() returns -1
test04: Expected 400, received res = -1 !!!
00000049A9BCFD78: async_operation::~async_operation(): m_value = -2
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo3b.exe):

```
--- test04: auxiliary function returns operation ---
0000008EE8CFF888: async_operation::async_operation(): m_value = -1
0000008EE8CFF888: start_operation::start_operation(20)
0000008EE8CFF888: start_operation::start_operation_impl(20)
EventHandler::set(..., 400)
EventHandler::run()
0000008EE8CFF888: async_operation::set_value(400)
0000008EE8CFF888: start_operation::get_value()
0000008EE8CFF888: async_operation::get_value() returns 400
test04: res = 400
0000008EE8CFF888: async_operation::~async_operation(): m_value = -2
```

The output of rvo3a.exe is incorrect, the output of rvo3b.exe is correct.

## Comparing compilers

The compiler that was used to produce the executables above, is the compiler of Visual Studio 2022.

When compiled with gcc 11.4.0 (on Ubuntu 22.04 LTS), RVO is used for all settings.

When the applications are built with Qt Creator 10.0.1, the debug versions did not use RVO, while the release versions did.

Therefore, the source code should be written in a way that produces correct output, irrespective of the use of RVO.

## Closing remarks

The output of all tests in rvo2.cpp is correct, independent of the use of RVO.

An additional advantage of using an index, 
is that this index can also be used for application-specific arrays or vectors
that complement the one used in async_ops.
