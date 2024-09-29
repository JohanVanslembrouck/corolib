# On the use of return value optimization (RVO)

The test applications in this directory investigate whether various compilers use return value optimization in a reliable and consistent way,
and if not, how we can deal with this.

## Definition

From https://en.wikipedia.org/wiki/Copy_elision:

In the context of the C++ programming language, return value optimization (RVO) is a compiler optimization 
that involves eliminating the temporary object created to hold a function's return value.
RVO is allowed to change the observable behaviour of the resulting program by the C++ standard.

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
```

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
    printf("test00: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test00: res = %d\n", ao.get_value());
}
```

Function test00() calls start_operation_impl() (see above) and passes the address of a local async_operation object, ao.
We run the event handler (eventHandler.run()), which will call the lambda function passed to it from start_operation_impl().
After that, we can obtain the result from the async_operation object by calling the member function get_value().

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test00: pass object as argument ---
0000004DCDAFFB18: async_operation::async_operation()
0000004DCDAFFB18 = ao: start_operation_impl
EventHandler::set(..., 400)
test00: &ao = 0000004DCDAFFB18
EventHandler::run()
0000004DCDAFFB18: async_operation::set_value(400)
0000004DCDAFFB18: async_operation::get_value()
test00: res = 400
0000004DCDAFFB18: async_operation::~async_operation()
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
    printf("test01: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test01: res = %d\n", ao.get_value());
}
```

It calls a new function start_operation(), which returns an async_operation object, ao.
The function start_operation() is defined as follows:

```c++
async_operation start_operation(int i) {
    async_operation ret;
    start_operation_impl(i, &ret);
    printf("%p = &ret: start_operation\n", &ret);
    return ret;
}
```

Function start_operation() creates a local async_operation object, ret,
passes its address to start_operation_impl(), and then returns ret.

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test01: test RVO ---
0000004DCDAFFAB8: async_operation::async_operation()
0000004DCDAFFAB8 = ao: start_operation_impl
EventHandler::set(..., 400)
0000004DCDAFFAB8 = &ret: start_operation
0000004DCDAFFB18: async_operation::async_operation(async_operation&&)
0000004DCDAFFAB8: async_operation::~async_operation()
test01: &ao = 0000004DCDAFFB18
EventHandler::run()
0000004DCDAFFAB8: async_operation::set_value(400)
0000004DCDAFFB18: async_operation::get_value()
test01: res = -1
0000004DCDAFFB18: async_operation::~async_operation()
```

This is the output when compiled with  MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test01: test RVO ---
00000002DE0FFCF8: async_operation::async_operation()
00000002DE0FFCF8 = ao: start_operation_impl
EventHandler::set(..., 400)
00000002DE0FFCF8 = &ret: start_operation
test01: &ao = 00000002DE0FFCF8
EventHandler::run()
00000002DE0FFCF8: async_operation::set_value(400)
00000002DE0FFCF8: async_operation::get_value()
test01: res = 400
00000002DE0FFCF8: async_operation::~async_operation()
```

The output of rvo1a.exe is wrong (it still contains the initial value -1 assigned by the constructor),
the output of rvo1b.exe is correct.

The second output shows that the compiler has used RVO, while in the first compilation,
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
    printf("test02: &ao = %p\n", &ao);
    eventHandler.run();
    printf("test02: res = %d\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test02: test RVO: immediate completion ---
0000004DCDAFFAB8: async_operation::async_operation()
0000004DCDAFFAB8 = ao: start_operation_impl
0000004DCDAFFAB8: async_operation::set_value(100)
0000004DCDAFFAB8 = &ret: start_operation
0000004DCDAFFB18: async_operation::async_operation(async_operation&&)
0000004DCDAFFAB8: async_operation::~async_operation()
test02: &ao = 0000004DCDAFFB18
EventHandler::run()
0000004DCDAFFB18: async_operation::get_value()
test02: res = 100
0000004DCDAFFB18: async_operation::~async_operation()
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test02: test RVO: immediate completion ---
00000002DE0FFCF8: async_operation::async_operation()
00000002DE0FFCF8 = ao: start_operation_impl
00000002DE0FFCF8: async_operation::set_value(100)
00000002DE0FFCF8 = &ret: start_operation
test02: &ao = 00000002DE0FFCF8
EventHandler::run()
00000002DE0FFCF8: async_operation::get_value()
test02: res = 100
00000002DE0FFCF8: async_operation::~async_operation()
```

The output of both applications is correct.
Although rvo1a.exe uses an intermediate object,
this object already contains the correct result at the moment it is "moved" to the final object in test02().

### test03: object goes out of scope

Function test03() is implemented as follows:

```c++
void test03a() {
    async_operation ao = start_operation(20);
    printf("test03a: &ao = %p\n", &ao);
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
0000004DCDAFFA88: async_operation::async_operation()
0000004DCDAFFA88 = ao: start_operation_impl
EventHandler::set(..., 400)
0000004DCDAFFA88 = &ret: start_operation
0000004DCDAFFAE8: async_operation::async_operation(async_operation&&)
0000004DCDAFFA88: async_operation::~async_operation()
test03a: &ao = 0000004DCDAFFAE8
0000004DCDAFFAE8: async_operation::~async_operation()
EventHandler::run()
0000004DCDAFFA88: async_operation::set_value(400)
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test03: object goes out of scope ---
00000002DE0FFCC8: async_operation::async_operation()
00000002DE0FFCC8 = ao: start_operation_impl
EventHandler::set(..., 400)
00000002DE0FFCC8 = &ret: start_operation
test03a: &ao = 00000002DE0FFCC8
00000002DE0FFCC8: async_operation::~async_operation()
EventHandler::run()
00000002DE0FFCC8: async_operation::set_value(400)
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
    printf("test04: res = %d\n", ao.get_value());
}
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo1a.exe):

```
--- test04: auxiliary function returns async_operation ---
0000004DCDAFFA58: async_operation::async_operation()
0000004DCDAFFA58 = ao: start_operation_impl
EventHandler::set(..., 400)
0000004DCDAFFA58 = &ret: start_operation
0000004DCDAFFAB8: async_operation::async_operation(async_operation&&)
0000004DCDAFFA58: async_operation::~async_operation()
test04a: &ao = 0000004DCDAFFAB8
0000004DCDAFFB18: async_operation::async_operation(async_operation&&)
0000004DCDAFFAB8: async_operation::~async_operation()
EventHandler::run()
0000004DCDAFFA58: async_operation::set_value(400)
0000004DCDAFFB18: async_operation::get_value()
test04: res = -1
0000004DCDAFFB18: async_operation::~async_operation()
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo1b.exe):

```
--- test04: auxiliary function returns async_operation ---
00000002DE0FFCF8: async_operation::async_operation()
00000002DE0FFCF8 = ao: start_operation_impl
EventHandler::set(..., 400)
00000002DE0FFCF8 = &ret: start_operation
test04a: &ao = 00000002DE0FFCF8
EventHandler::run()
00000002DE0FFCF8: async_operation::set_value(400)
00000002DE0FFCF8: async_operation::get_value()
test04: res = 400
00000002DE0FFCF8: async_operation::~async_operation()
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

Empty implementation.

### test01: test RVO

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test01: test RVO ---
00000085B12FFCB8: async_operation::async_operation(0)
00000085B12FFCB8 = &ret: start_operation, idx = 0
EventHandler::set(..., 400)
00000085B12FFD18: async_operation::async_operation(async_operation&&)
00000085B12FFCB8: async_operation::~async_operation(-1)
test01: &ao = 00000085B12FFD18
EventHandler::run()
start_operation_impl: index = 0, ao = 00000085B12FFD18
00000085B12FFD18: async_operation::set_value(400)
00000085B12FFD18: async_operation::get_value()
test01: res = 400
00000085B12FFD18: async_operation::~async_operation(0)
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test01: test RVO ---
0000004DE58FF7C8: async_operation::async_operation(0)
0000004DE58FF7C8 = &ret: start_operation, idx = 0
EventHandler::set(..., 400)
test01: &ao = 0000004DE58FF7C8
EventHandler::run()
start_operation_impl: index = 0, ao = 0000004DE58FF7C8
0000004DE58FF7C8: async_operation::set_value(400)
0000004DE58FF7C8: async_operation::get_value()
test01: res = 400
0000004DE58FF7C8: async_operation::~async_operation(0)
```

The output of both is correct.

### test02: test RVO: immediate completion

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test02: test RVO: immediate completion ---
00000085B12FFCB8: async_operation::async_operation(1)
00000085B12FFCB8 = &ret: start_operation, idx = 1
start_operation_impl: index = 1, ao = 00000085B12FFCB8
00000085B12FFCB8: async_operation::set_value(100)
00000085B12FFD18: async_operation::async_operation(async_operation&&)
00000085B12FFCB8: async_operation::~async_operation(-1)
test02: &ao = 00000085B12FFD18
EventHandler::run()
00000085B12FFD18: async_operation::get_value()
test02: res = 100
00000085B12FFD18: async_operation::~async_operation(1)
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test02: test RVO: immediate completion ---
0000004DE58FF7C8: async_operation::async_operation(1)
0000004DE58FF7C8 = &ret: start_operation, idx = 1
start_operation_impl: index = 1, ao = 0000004DE58FF7C8
0000004DE58FF7C8: async_operation::set_value(100)
test02: &ao = 0000004DE58FF7C8
EventHandler::run()
0000004DE58FF7C8: async_operation::get_value()
test02: res = 100
0000004DE58FF7C8: async_operation::~async_operation(1)
```

The output of both is correct.

### test03: object goes out of scope

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 11)' in CMakeLists.txt (executable rvo2a.exe):

```
--- test03: object goes out of scope ---
00000085B12FFC88: async_operation::async_operation(2)
00000085B12FFC88 = &ret: start_operation, idx = 2
EventHandler::set(..., 400)
00000085B12FFCE8: async_operation::async_operation(async_operation&&)
00000085B12FFC88: async_operation::~async_operation(-1)
test03a: &ao = 00000085B12FFCE8
00000085B12FFCE8: async_operation::~async_operation(2)
EventHandler::run()
start_operation_impl: index = 2, ao = 0000000000000000
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test03: object goes out of scope ---
0000004DE58FF798: async_operation::async_operation(2)
0000004DE58FF798 = &ret: start_operation, idx = 2
EventHandler::set(..., 400)
test03a: &ao = 0000004DE58FF798
0000004DE58FF798: async_operation::~async_operation(2)
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
00000085B12FFC58: async_operation::async_operation(3)
00000085B12FFC58 = &ret: start_operation, idx = 3
EventHandler::set(..., 400)
00000085B12FFCB8: async_operation::async_operation(async_operation&&)
00000085B12FFC58: async_operation::~async_operation(-1)
test04a: &ao = 00000085B12FFCB8
00000085B12FFD18: async_operation::async_operation(async_operation&&)
00000085B12FFCB8: async_operation::~async_operation(-1)
EventHandler::run()
start_operation_impl: index = 3, ao = 00000085B12FFD18
00000085B12FFD18: async_operation::set_value(400)
00000085B12FFD18: async_operation::get_value()
test04: res = 400
00000085B12FFD18: async_operation::~async_operation(3)
```

This is the output when compiled with MSVC with 'set(CMAKE_CXX_STANDARD 20)' in CMakeLists.txt (executable rvo2b.exe):

```
--- test04: auxiliary function returns async_operation ---
0000004DE58FF7C8: async_operation::async_operation(3)
0000004DE58FF7C8 = &ret: start_operation, idx = 3
EventHandler::set(..., 400)
test04a: &ao = 0000004DE58FF7C8
EventHandler::run()
start_operation_impl: index = 3, ao = 0000004DE58FF7C8
0000004DE58FF7C8: async_operation::set_value(400)
0000004DE58FF7C8: async_operation::get_value()
test04: res = 400
0000004DE58FF7C8: async_operation::~async_operation(3)
```

The output of both is correct.

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
