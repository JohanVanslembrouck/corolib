# Why use C++ coroutines for (distributed) applications?

This directory contains various examples that explain the advantage of C++ coroutines 
for writing (distributed) applications.

Consider a program name p1XYZ-purpose-of-the-program.cpp:

* X stands for programs with the same functionality for the same value of X.
* Y = 0 or Y = 5 is used for the synchronous version, Y = 1 or Y = 6 is used for the asynchronous version and Y = 2 or Y = 7 is used for the coroutine version.
* Z = 0 is used for the base version. Variants are numbered 2, 4, etc.

The main difference between the Y = 0, 1, 2 variants on the one hand, and the Y = 5, 6, 7 variants on the other hand,
is that the latter variants use (dummy) implementations for the write and read functions.
These implementations are missing in the former variants.

No real remote method invocations (RMIs) are used in the examples.
Instead, the delay that may result of the invocation is simulated by means of a timer.

## Case 1: program with 1 RMI

This section compares three variants of a function or coroutine with 1 RMI in its middle, between a part 1 and a part 2.

### p1000-sync-1rmi.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    int function1(int in1, int in2)
	{
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1.op1(in1, in2, out1, out2);
        printf("Class01::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + in2 + out1 + out2 + ret1;
    }
};
```

Because of the synchronous RMI, it may take a while before the RMI returns.
During that time, the program cannot handle other events.

```c++
int main()
{
    printf("main();\n");
    int ret1 = class01.function1(11, 12);
    int ret2 = class01.function1(21, 22);
    int ret3 = class01.function1(31, 32);
    int ret4 = class01.function1(41, 42);
    printf("\n");

    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    printf("main(): ret3 = %d\n", ret3);
    printf("main(): ret4 = %d\n", ret4);
    return 0;
}
```

### p1002-sync+thread-1rmi.cpp

It is possible to make the program reactive by running every function on its own thread.
The implementation of function1 does not have to be changed. This can be accomplished as follows:

```c++
int main()
{
    printf("main();\n");
    std::future<int> t1 = std::async(std::launch::async, []() { return class01.function1(11, 12); });
    int ret1 = t1.get();
    std::future<int> t2 = std::async(std::launch::async, []() { return class01.function1(21, 22); });
    int ret2 = t2.get();
    printf("\n");

    printf("main(): ret1 = %d\n", ret1);
    printf("main(): ret2 = %d\n", ret2);
    return 0;
}
```

### p1010-async-1rmi.cpp

The application class now looks as follows:

```c++
class Class01
{
public:
    
    struct function1_cxt_t
    {
        ~function1_cxt_t() { printf("function1_cxt_t::~function1_cxt_t()\n"); }

        int in1;
        int in2;
        int* ret;
    };

    /**
     * @brief
     *
     */
    void function1(int in1, int in2, int& ret)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, ret = %d)\n", in1, in2, ret);
        void* ctxt = new function1_cxt_t{ in1, in2, &ret };
        printf("Class01::function1(): ctxt = %p\n", ctxt);

        remoteObj1.sendc_op1(ctxt, in1, in2,
            [this](void* context, int out1, int out2, int ret1)
            {
                this->function1_cb(context, out1, out2, ret1);
            });
    }

    /**
     * @brief callback function with the out parameters and the return value
     *
     */
    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Class01::function1_cb(context = %p, out1 = %d, out2 = %d, ret1 = %d)\n", context, out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);

        printf("Class01::function1_cb(): ctxt->in1 = %d, ctxt->in2 = %d, ctxt->ret = %p, ctxt->ret = %d)\n", 
            ctxt->in1, ctxt->in2, ctxt->ret, *ctxt->ret);
        *ctxt->ret = ctxt->in1 + ctxt->in2 + out1 + out2 + ret1;
        delete ctxt;
    }
```

function1() returns control to the event loop as soon as it has started the remote operation by calling sendc_op1.
When that operation completes, function1_cb is called from the event loop.

### p1020-coroutines-1rmi.cpp

The application class now looks as follows:

```c++
class Class01
{
public:
    async_task<int> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d)\n", in1, in2);
        int out1 = -1, out2 = -1;
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + in2 + out1 + out2 + ret1;
    }
};
```

It is very close to the synchronous implementation, but it is reactive.

### p1050-sync-1rmi.cpp

The application code is the same as that of p1000-sync-1rmi.cpp.
Only 3 lines are different:

```c++
#include "p1050.h"                              // difference with p1000-sync-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1000-sync-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1000-sync-1rmi.cpp
```

### p1060-async-1rmi.cpp

The application code is the same as that of p1010-async-1rmi.cpp.
Only 3 lines are different:

```c++
#include "p1050.h"                              // difference with p1010-async-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1010-async-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1010-async-1rmi.cpp

```

### p1070-coroutines-1rmi.cpp

The application code is the same as that of p1020-coroutines-1rmi.cpp.
Only 4 lines are different:

```c++
#include "p1050co.h"                                        // difference with p1020-coroutine-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                             // difference with p1020-coroutine-1rmi.cpp
RemoteObjectImplCo remoteObjImplco{ remoteObjImpl };        // difference with p1020-coroutine-1rmi.cpp
RemoteObject1Co remoteObj1co{ remoteObjImplco };            // difference with p1020-coroutine-1rmi.cpp
```

## Case 2: program with callstack and 1 RMI

The remote operation can be invoked from a function called from another function called from another function, etc.
These functions form a call stack as can be found in protocol stacks, application stacks, device driver stacks, etc.

This section describes ways to make the original synchronous application reactive again. 

### p1100-sync-callstack-1rmi.cpp

The three application classes look as follows:

```c++
class Layer01
{
public:
    int function1(int in1, int& out1, int& out2)
    {
        printf("Layer01::function1(in1 = %d, out1 = %d, out2 = %d)\n", in1, out1, out2);
        int ret1 = remoteObj1.op1(in1, in1, out1, out2);
        printf("Layer01::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + ret1;
    }
};

Layer01 layer01;

class Layer02
{
public:
    int function1(int in1, int& out1)
    {
        printf("Layer02::function1(in1 = %d, out1 = %d)\n", in1, out1);
        int out2 = -1;
        int ret1 = layer01.function1(in1, out1, out2);
        printf("Layer02::function1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        return in1 + out2 + ret1;
    }
};

Layer02 layer02;

class Layer03
{
public:
    int function1(int in1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = layer02.function1(in1, out1);
        printf("Layer03::function1(): out1 = %d, ret1 = %d\n", out1, ret1);
        return in1 + out1 + ret1;
    }
};

Layer03 layer03;
```

### p1110-async-callstack-1rmi.cpp

The three application classes now look as follows:

```c++
class Layer01
{
public:
    struct function1_cxt_t
    {
        void* ctxt;
        lambda_vp_3int_t lambda;
        int in1;
    };

    void function1(void* ctxt1, lambda_vp_3int_t lambda, int in1)
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ ctxt1, lambda, in1 };

        // int ret1 = remoteObj1.op1(in1, in1, out1, out2);
        remoteObj1.sendc_op1(ctxt, in1, in1,
            [this](void* context, int out1, int out2, int ret1)
            {
                this->function1_cb(context, out1, out2, ret1);
            });
    }

    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Layer01::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // call function1_cb of upper layer
        // return in1 + ret1;
        ctxt->lambda(ctxt->ctxt, out1, out2, ctxt->in1 + ret1);
        delete ctxt;
    }
};

Layer01 layer01;

class Layer02
{
public:
    struct function1_cxt_t
    {
        void* ctxt;
        lambda_vp_2int_t lambda;
        int in1;
    };

    void function1(void* ctxt1, lambda_vp_2int_t lambda, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ ctxt1, lambda, in1 };

        // int ret1 = layer01.function1(in1, out1, out2);
        layer01.function1(
            ctxt,
            [this](void* ctxt1, int out1, int out2, int ret1) { 
                this->function1_cb(ctxt1, out1, out2, ret1);
            },
            in1);
    }

    void function1_cb(void* context, int out1, int out2, int ret1)
    {
        printf("Layer02::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // call function1_cb of upper layer
        // return in1 + out2 + ret1;
        ctxt->lambda(ctxt->ctxt, out1, ctxt->in1 + out2 + ret1);
        delete ctxt;
    }
};

Layer02 layer02;

class Layer03
{
public:
    struct function1_cxt_t
    {
        int* ret;
        int in1;
    };

    void function1(int in1, int& ret1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ &ret1, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            [this](void *ctxt, int out1, int ret1) {
                this->function1_cb(ctxt, out1, ret1);
            },
            in1);
    }

    void function1_cb(void* context, int out1, int ret1)
    {
        printf("Layer03::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // return in1 + out1 + ret1;
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
    }

    struct function2_cxt_t
    {
        int* ret;
        int in1;
    };

    void function2(int in1, int& ret1)
    {
        printf("Layer03::function2(in1 = %d)\n", in1);
        void* ctxt = new function1_cxt_t{ &ret1, in1 };

        // int ret1 = layer02.function1(in1, out1);
        layer02.function1(ctxt,
            [this](void* ctxt, int out1, int ret1) {
                this->function2_cb(ctxt, out1, ret1);
            },
            in1);
    }

    void function2_cb(void* context, int out1, int ret1)
    {
        printf("Layer03::function2_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        function1_cxt_t* ctxt = static_cast<function1_cxt_t*>(context);
        // return in1 + out1 + ret1;
        *ctxt->ret = ctxt->in1 + out1 + ret1;
        delete ctxt;
    }
};

Layer03 layer03;
```

### p1112-async-callstack-1rmi-queue-cs.cpp

This variant usses a callstack variable that is passed from the upper layer (Layer03)
to the lower layer (Layer01) and to the remote object.
This way, it is possible to find the way back upwards the call stack without relying on a single data member.
Several RMIs can be invoked one after the other, because they will use a dedicated callstack.

The three application classes look as follows:

```c++
class Layer01
{
public:
    void function1(CallStack& callstack, int in1) 
    {
        printf("Layer01::function1(in1 = %d)\n", in1);
        lambda_cs_3int_t* op = new lambda_cs_3int_t(
            [this](CallStack& callstack, int out1, int out2, int ret1)
            {
                this->function1_cb(callstack, out1, out2, ret1);
            });
        callstack.push(op);
        remoteObj1.sendc_op1(callstack, in1, in1);
        printf("Layer01::function1(): return\n");
    }

    void function1_cb(CallStack& callstack, int out1, int out2, int ret1) 
    {
        printf("Layer01::function1_cb(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        // call function1_cb of upper layer (Layer02)
        lambda_cs_2int_t* op = static_cast<lambda_cs_2int_t*>(callstack.top_pop());
        (*op)(callstack, out1, ret1);
        printf("Layer01::function1_cb(): delete %p\n", op);
        delete op;
    }
};

Layer01 layer01;

class Layer02
{
public:    
    void function1(CallStack& callstack, int in1)
    {
        printf("Layer02::function1(in1 = %d)\n", in1);
        lambda_cs_2int_t* op = new lambda_cs_2int_t(
            [this](CallStack& callstack, int out1, int ret1)
            {
                this->function1_cb(callstack, out1, ret1);
            });
        callstack.push(op);
        layer01.function1(callstack, in1);
        printf("Layer02::function1(): return\n");
    }

    void function1_cb(CallStack& callstack, int out1, int ret1)
    {
        printf("Layer02::function1_cb(out1 = %d, ret1 = %d)\n", out1, ret1);
        // call function1_cb of upper layer (Layer03)
        lambda_cs_1int_t* op = static_cast<lambda_cs_1int_t*>(callstack.top_pop());
        (*op)(callstack, ret1);
        printf("Layer02::function1_cb(): delete %p\n", op);
        delete op;
    }
};

Layer02 layer02;

class Layer03
{
public:       
    void function1(int in1)
    {
        printf("Layer03::function1(in1 = %d)\n", in1);
        lambda_cs_1int_t* p = new lambda_cs_1int_t(
            [this](CallStack& callstack, int ret1)
            {
                this->function1_cb(callstack, ret1);
            });
        m_callstack1.push(p);
        layer02.function1(m_callstack1, in1);
        printf("Layer03::function1(): return\n");
    }

    void function1_cb(CallStack&, int ret1)
    {
        printf("Layer03::function1_cb(ret1 = %d)\n", ret1);
    }
    
    void function2(int in1)
    {
        printf("Layer03::function2(in1 = %d)\n", in1);
        lambda_cs_1int_t* p = new lambda_cs_1int_t(
            [this](CallStack& callstack, int ret1)
            {
                this->function2_cb(callstack, ret1);
            });
        m_callstack2.push(p);
        layer02.function1(m_callstack2, in1);
    }

    void function2_cb(CallStack&, int ret1)
    {
        printf("Layer03::function2_cb(ret1 = %d)\n", ret1);
    }

private:
    CallStack m_callstack1;
    CallStack m_callstack2;
};

Layer03 layer03;
```

The implementation is rather complex and it uses dynamic memory.

### p1120-coroutines-callstack-1rmi.cpp

The three application classes look as follows:

```c++
class Layer01
{
public:
    async_task<int> coroutine1(int in1, int& out1, int& out2)
    {
        printf("Layer01::coroutine1(in1 = %d, out1 = %d, out2 = %d)\n", in1, out1, out2);
        int ret1 = co_await remoteObj1co.op1(in1, in1, out1, out2);
        printf("Layer01::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + ret1;
    }
};

Layer01 layer01;

class Layer02
{
public:
    async_task<int> coroutine1(int in1, int& out1)
    {
        printf("Layer01::coroutine1(in1 = %d, out1 = %d)\n", in1, out1);
        int out2 = -1;
        int ret1 = co_await layer01.coroutine1(in1, out1, out2);
        printf("Layer02::coroutine1(): out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        co_return in1 + out2 + ret1;
    }
};

Layer02 layer02;

class Layer03
{
public:
    async_task<int> coroutine1(int in1)
    {
        printf("Layer03::coroutine1(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = co_await layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine1(): out1 = %d, ret1 = %d\n", out1, ret1);
        co_return in1 + out1 + ret1;
    }

    async_task<int> coroutine2(int in1)
    {
        printf("Layer03::coroutine2(in1 = %d)\n", in1);
        int out1 = -1;
        int ret1 = co_await layer02.coroutine1(in1, out1);
        printf("Layer03::coroutine2(): out1 = %d, ret1 = %d\n", out1, ret1);
        co_return in1 + out1 + ret1;
    }
};

Layer03 layer03;
```

The coroutine mechanism takes care of returning control to the upper layers, 
even in case several RMIs are invoked one after the other.

### p1150-sync-callstack-1rmi.cpp

The application code is the same as that of p1100-sync-callstack-1rmi.cpp.
Only 3 lines are different:

```c++
#include "p1050.h"                              // difference with p1100-sync-callstack-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1100-sync-callstack-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1100-sync-callstack-1rmi.cpp

```

### p1160-async-callstack-1rmi.cpp

The application code is the same as that of p1110-async-callstack-1rmi.cpp.
Only 3 lines are different:

```c++
#include "p1050.h"                              // difference with p1110-async-callstack-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1110-async-callstack-1rmi.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1110-async-callstack-1rmi.cpp
```

### p1170-coroutines-callstack-1rmi.cpp

The application code is the same as that of p1120-coroutines-callstack-1rmi.cpp.
Only 4 lines are different:

```c++
#include "p1050co.h"                                        // difference with p1120-coroutines-callstack-1rmi.cpp

RemoteObjectImpl remoteObjImpl;                             // difference with p1120-coroutines-callstack-1rmi.cpp
RemoteObjectImplCo remoteObjImplco{ remoteObjImpl };        // difference with p1120-coroutines-callstack-1rmi.cpp
RemoteObject1Co remoteObj1co{ remoteObjImplco };            // difference with p1120-coroutines-callstack-1rmi.cpp
```

## Case 3: program with if-then-else and 3 RMIs

In this section we use a function with 3 RMIs and an if-then-else. Depending on the result of the first RMI,
we enter the 'then' or the 'else' case, where another remote method is invoked.

### p1200-sync-3rmis.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    void function1(int in1, int in2, int testval)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        int ret1 = remoteObj1.op1(in1, in2, out1, out2);
        printf("Class01::function: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1 Do stuff
        if (ret1 == testval) {
            int out3 = -1;
            int ret2 = remoteObj2.op2(in1, in2, out3);
            printf("Class01::function: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = remoteObj3.op3(in1, out4, out5);
            printf("Class01::function: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3 Do stuff
        }
    }
};

Class01 class01;

int main()
{
    printf("main();\n");
    class01.function1(11, 12, 10);
    printf("\n");
    class01.function1(11, 12, 23);
    return 0;
}
```

### p1202-sync+thread-3rmis.cpp

It is possible to make the program reactive by running every function on its own thread.
The implementation of function1 does not have to be changed. This can be done as follows:

```c++
int main()
{
    printf("main();\n");
    std::thread th1(&Class01::function1, &class01, 11, 12, 10); th1.join();
    std::thread th2(&Class01::function1, &class01, 11, 12, 23); th2.join();
    return 0;
}
```

### p1210-async-3rmis.cpp

The application class now looks as follows:

```c++
class Class01
{
public:
    void function1(int in1, int in2, int testval)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2, 
            [this, in1, in2, testval](int out1, int out2, int ret1) {
                this->function1a(in1, in2, testval, out1, out2, ret1);
            });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int in1, int in2, int testval, int out1, int out2, int ret1)
    {
        printf("Class01::function1a(out1 = %d, out2 = %d, ret1 = %d, testval = %d)\n", out1, out2, ret1, testval);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == testval) {
            remoteObj2.sendc_op2(in1, in2,
                [this](int out1, int ret1) {
                    this->function1b(out1, ret1);
                });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in1,
                [this](int out1, int out2, int ret1) {
                    this->function1c(out1, out2, ret1);
                });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out3, int ret2)
    {
        printf("Class01::function1b(out3 = %d, ret2 = %d)\n", out3, ret2);
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out4, int out5, int ret3)
    {
        printf("Class01::function1c(out4 = %d, out5 = %d, ret3 = %d)\n", out4, out5, ret3);
        // 3b Do stuff that needs the result of the RMI
    }

    /**
     * @brief alternative version of function1 that avoids the introduction of callback functions
     * by placing the original code in lambdas.
     *
     */
    void function1alt(int in1, int in2, int testval)
    {
        printf("Class01::function1alt(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2, 
            [this, in1, in2, testval](int out1, int out2, int ret1)
            { 
                printf("Class01::function1alt: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
                // 1b Do stuff that needs the result of the RMI
                if (ret1 == testval) {
                    remoteObj2.sendc_op2(in1, in2,
                        [this](int out3, int ret2)
                        {
                            printf("Class01::function1alt: 2: out3 = %d, ret2 = %d\n", out3, ret2);
                            // 2b Do stuff that needs the result of the RMI
                        });
                    // 2a Do stuff that doesn't need the result of the RMI
                }
                else {
                    remoteObj3.sendc_op3(in1,
                        [this](int out4, int out5, int ret3) 
                        {
                            printf("Class01::function1alt: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
                            // 3b Do stuff that needs the result of the RMI
                        });
                    // 3a Do stuff that doesn't need the result of the RMI
                }
            });
        // 1a Do stuff that doesn't need the result of the RMI
    }
};
```

### p1212-async-3rmis-local-event-loop.cpp

This example uses a local event loop that is placed close after the invocation of an asychronous variant
of the original two-way in-out RMI.

The local event loop can only handle the callback function passed to the asynchronous function.

The application class looks as follows:

```c++
class Class01
{
public:
    void function1(int in1, int in2, int testval)
    {
        int lret1 = -1;
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2,
            [this, &lret1](int out1, int out2, int ret1)
            {
                lret1 = this->callback1(out1, out2, ret1);
            });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (lret1 == testval) {
            remoteObj2.sendc_op2(in1, in2, 
                [this](int out1, int ret1)
                {
                    this->callback2(out1, ret1);
                });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in1, 
                [this](int out1, int out2, int ret1)
                {
                    this->callback3(out1, out2, ret1);
                });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

    int callback1(int out1, int out2, int ret1)
    { 
        printf("Class01::callback1(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        // copy to local variables
        return ret1;
    }
    
    void callback2(int out3, int ret2) { 
        printf("Class01::callback2(out3 = %d, ret2 = %d)\n", out3, ret2);
        // copy to local variables
    }
    
    void callback3(int out4, int out5, int ret3) {
        printf("Class01::callback3(out4 = %d, out5 = %d, ret3 = %d)\n", out4, out5, ret3);
    }
};
```

### p1220-coroutines-3rmis.cpp

The application class looks as follows:

```c++
struct Class01
{
    async_task<void> coroutine1(int in1, int in2, int testval)
    {
        printf("Class01::coroutine1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        int ret1 = co_await remoteObj1co.op1(in1, in2, out1, out2);
        printf("Class01::coroutine1: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1 Do stuff
        if (ret1 == testval) {
            int out3 = -1;
            int ret2 = co_await remoteObj2co.op2(in1, in2, out3);
            printf("Class01::coroutine1: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2 Do stuff
        }
        else {
            int out4 = -1, out5 = -1;
            int ret3 = co_await remoteObj3co.op3(in1, out4, out5);
            printf("Class01::coroutine1: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3 Do stuff
        }
    }
    
    async_task<void> coroutine1a(int in1, int in2, int testval)
    {
        printf("Class01::coroutine1a(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        int out1 = -1, out2 = -1;
        async_task<int> op1 = remoteObj1co.op1(in1, in2, out1, out2);
        // 1a Do some stuff that doesn't need the result of the RMI
        int ret1 = co_await op1;
        printf("Class01::coroutine1a: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == testval) {
            int out3 = -1;
            async_task<int> op2 = remoteObj2co.op2(in1, in2, out3);
            // 2a Do some stuff that doesn't need the result of the RMI
            int ret2 = co_await op2;
            printf("Class01::coroutine1a: 2: out3 = %d, ret2 = %d\n", out3, ret2);
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            int out4 = -1, out5 = -1;
            async_task<int> op3 = remoteObj3co.op3(in1, out4, out5);
            // 3a Do some stuff that doesn't need the result of the RMI
            int ret3 = co_await op3;
            printf("Class01::coroutine1a: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
            // 3b Do stuff that needs the result of the RMI
        }
    }
};
```

## Case 4: programa with 3 "parallel" RMIs

An application may have to interact with several remote applications in parallel,
e.g. to collect information from these applications and process that information afterwards.

In the synchronous approach, the interactions with these remote applications has to performed
in a sequential order. The total delay is the sum of the delay of the individual interactions (RMIs).
It would be much better if these RMIs can run in parallel, especially of the remote applications
run on different processors or cores.

It is more efficient, of course, to start the remote operations one after the other and
collect the responses when they come in. This is possible with the asynchronous approach
and with coroutines. The total delay will be reduced to about the time of a single remote interaction.

### p1500-sync-3-parallel-rmis.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;
  
        int ret1 = remoteObj1.op1(in1, in2, out11, out12);
        int ret2 = remoteObj2.op1(in1, in2, out21, out22);
        int ret3 = remoteObj3.op1(in1, in2, out31, out32);
        int result = ret1 + ret2 + ret3;
        printf("Class01::function1(): result = %d\n", result);
    }
};
```

### p1510-async-3-parallel-rmis.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(0, out1, out2, ret1); });
        remoteObj2.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(1, out1, out2, ret1); });
        remoteObj3.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(2, out1, out2, ret1); });
    }

    void function1a(int index, int out1, int out2, int ret1)
    {
        printf("Class01::function1a(%d, %d, %d)\n", out1, out2, ret1);
        callfinished[index] = true;
        result[index] = ret1;
        if (callfinished[0] && callfinished[1] && callfinished[2])
            printf("Class01::function1a: result = %d\n", result[0] + result[1] + result[2]);
    }
    
private:
    bool callfinished[3]{ false, false, false };
    int result[3]{ 0, 0, 0 };
};
```

### p1520-coroutines-3-parallel-rmis.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    async_task<void> coroutine1(int in1, int in2)
    {
        printf("Class01::coroutine1()\n");
        int out11 = -1, out12 = -1;
        int out21 = -1, out22 = -1;
        int out31 = -1, out32 = -1;

        async_task<int> op1 = remoteObj1co.op1(in1, in2, out11, out12);
        async_task<int> op2 = remoteObj2co.op1(in1, in2, out21, out21);
        async_task<int> op3 = remoteObj3co.op1(in1, in2, out31, out32);
#if 0
        // The following statement does not compile with g++ 11.3.0
        co_await when_all({ &op1, &op2, &op3 });
#else
        when_all wa({ &op1, &op2, &op3 });
        co_await wa;
#endif
        printf("Class01::coroutine1(): result = %d\n", op1.get_result() +  op2.get_result() + op3.get_result());
    } // g++ 11 reports at this line: error: array used as initializer
};
```

## Case 5: program with nested for loop

In this section we consider a function with a nested for-loop (i.e. a for-loop inside another for-loop):
* The outer for-loop iterates over message lengths from 0 to MAX_MSG_LENGTH.
* The inner for-loop then invokes the RMI NR_MSGS_TO_SEND times.

This results in MAX_MSG_LENGTH * NR_MSGS_TO_SEND RMIs.

### p1300-sync-nested-loop.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    void function1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class04::function1(): i = %d\n", i);
            Msg msg(i * 10);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class04::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                int ret1 = remoteObj1.op1(msg);
                (void)ret1;
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};
```

### p1310-async-nested-loop.cpp

The application class now looks as follows:

```c++
class Class01
{
public:
    void function1()
    {
        printf("Class01::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a()
    {
        printf("Class01::function1a(): counter = %d\n", counter);
        if (j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < MAX_MSG_LENGTH) {
                msg = Msg(i * 10);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
    }
    
private:
    int i = 0, j = 0;
    Msg msg;
    int counter = 0;
};
```

### p1320-coroutines-nested-loop.cpp

The application class looks as follows:

```c++
class Class01
{
public:
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::coroutine1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::coroutine1(): i = %d\n", i);
            Msg msg(i * 10);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class02::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_operation<int> op1 = remoteObj1co.start_op1(msg);
                int i = co_await op1;
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};
```

### p1350-sync-nested-loop.cpp

The application code is the same as that of p1300-sync-nested-loop.cpp.
Only 3 lines are different:

```c++
#include "p1350.h"                              // difference with p1300-sync-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1300-sync-nested-loop.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1300-sync-nested-loop.cpp
```

### p1360-async-nested-loop.cpp

The application code is the same as that of p1310-async-nested-loop.cpp.
Only 3 lines are different:

```c++
#include "p1350.h"                              // difference with p1310-async-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                 // difference with p1310-async-nested-loop.cpp
RemoteObject1 remoteObj1{ remoteObjImpl };      // difference with p1310-async-nested-loop.cpp
```

### p1370-coroutines-nested-loop.cpp

The application code is the same as that of p1320-coroutines-nested-loop.cpp.
Only 4 lines are different:

```c++
#include "p1350co.h"                                        // difference with p1320-coroutines-nested-loop.cpp

RemoteObjectImpl remoteObjImpl;                             // difference with p1320-coroutines-nested-loop.cpp
RemoteObjectImplCo remoteObjImplco{ remoteObjImpl };        // difference with p1320-coroutines-nested-loop.cpp
RemoteObject1Co remoteObj1co{ remoteObjImplco };            // difference with p1320-coroutines-nested-loop.cpp
```

## Case 6: Segmentation: adding two loops at the infrastructure level

This section contains examples that expand on the Case 4 examples.

A (large) request and response to and from the remote server cannot be written or read in a single packet.
Instead, a request or response has to be split into segments with a fixed maximum length of SEGMENT_LENGTH.

The original RMI code from the previous examples are in fact split into two for-loops:
* The first for-loop splits the request buffer into segments that are transmitted to the server using a lower-level write RMI.
* The second for-loop reads the response in segments until the whole response has been received: it uses a lower-level read RMI.

This thus leads to two for-loops inside a for-loop inside a for-loop:
* The outer for-loop iterates over message lengths from 0 to MAX_MSG_LENGTH.
* The middle for-loop invokes the higher-level RMI NR_MSGS_TO_SEND times.
* The two inner for-loops are for writing and reading the request and response, respectively.

Notice that the current implementation uses output messages with a fixed length of 55 and input messages with a fixed length of 35.

### p1400-sync-segmentation.cpp

The implementation of op1 looks as follows:

```c++
class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
        : m_remoteObjImpl(remoteObjImpl)
    {
    }

    Msg op1(Msg msg)
    {
        // Write part
        Buffer writebuffer;
        printf("RemoteObject1::op1()\n");
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        int buflength = writebuffer.length();
        for (int offset = 0; offset < buflength; offset += SEGMENT_LENGTH)
        {
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            printf("RemoteObject1::op1(): calling write_segment: offset = %d\n", offset);
            m_remoteObjImpl.write_segment(writebuffer.buffer(), offset, bytestowrite);
        }

        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1::op1(): calling read_segment: offset = %d\n", offset);
            completed = m_remoteObjImpl.read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        return res;
    }

    // ...

protected:
    RemoteObjectImpl& m_remoteObjImpl;
};
```

The application class looks as follows:

```c++
class Class01
{
public:
    void function1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                Msg res = remoteObj1.op1(msg);
                // Do something with msg
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};
```

### p1410-async-segmentation.cpp

The implementation of sendc_op1 now looks as follows:

```c++
class RemoteObject1
{
public:
    RemoteObject1(RemoteObjectImpl& remoteObjImpl)
        : m_remoteObjImpl(remoteObjImpl)
    {
    }

    // ...

    struct op1_context
    {
        int offset = 0;
        Buffer writebuffer;
        Buffer readbuffer;
        lambda_msg_t lambda;
    };

    void sendc_op1(Msg msg, lambda_msg_t op1_cb)
    {
        printf("RemoteObject1::sendc_op1(): calling write_segment\n");

        op1_context* ctxt = new op1_context;
        ctxt->lambda = op1_cb;
        //m_lambda = op1_cb;

        // Write part
        // Marshall msg into writebuffer
        // (code not present)
        // Write the first segment
        int buflength = ctxt->writebuffer.length();
        int bytestowrite = (buflength - ctxt->offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - ctxt->offset;
        m_remoteObjImpl.sendc_write_segment(ctxt->writebuffer.buffer(), ctxt->offset, bytestowrite,
            [this, ctxt]() { this->handle_write_segment_op1(ctxt); });
        ctxt->offset += SEGMENT_LENGTH;
    }

    void handle_write_segment_op1(op1_context *ctxt)
    {
        int buflength = ctxt->writebuffer.length();
        if (ctxt->offset < buflength) {
            printf("RemoteObject1::handle_write_segment_op1(%p): calling sendc_write_segment\n", ctxt);
            int bytestowrite = (buflength - ctxt->offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - ctxt->offset;
            m_remoteObjImpl.sendc_write_segment(ctxt->writebuffer.buffer(), ctxt->offset, bytestowrite,
                [this, ctxt]() { this->handle_write_segment_op1(ctxt); });
            ctxt->offset += SEGMENT_LENGTH;
        }
        else {
            // Read part
            ctxt->offset = 0;
            printf("RemoteObject1::handle_write_segment_op1(%p): calling sendc_read_segment\n", ctxt);
            m_remoteObjImpl.sendc_read_segment(ctxt->readbuffer.buffer(), ctxt->offset, SEGMENT_LENGTH,
                [this, ctxt](bool res) { this->handle_read_segment_op1(ctxt, res); });
            ctxt->offset += SEGMENT_LENGTH;
        }
    }

    void handle_read_segment_op1(op1_context* ctxt, bool complete)
    {
        printf("RemoteObject1::handle_read_segment_op1(%p, %d): calling sendc_read_segment\n", ctxt, complete);
        Msg msg;
        if (!complete) {
            m_remoteObjImpl.sendc_read_segment(ctxt->readbuffer.buffer(), ctxt->offset, SEGMENT_LENGTH,
                [this, ctxt](bool res) { this->handle_read_segment_op1(ctxt, res); });
            ctxt->offset += SEGMENT_LENGTH;
        }
        else {
            // Unmarshall msg from buf
            // (code not present)
            // Invoke the lambda passing the result
            ctxt->lambda(msg);
            delete ctxt;
        }
    }

protected:
    RemoteObjectImpl& m_remoteObjImpl;
};
```

The application class now looks as follows:

```c++
class Class01
{
public:
    void function1()
    {
		counter = 0;
        printf("Class01::function1(): counter = %d\n", counter);
        i = j = 0;
        
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
    }

    void function1a(Msg msgout)
    {
        // Do something with msgout
        printf("Class01::function1a(Msg): counter = %d\n", counter);
        if (j < NR_MSGS_TO_SEND) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
            j++;
            counter++;
        }
        else {
            // End of inner loop
            j = 0;
            i++;
            if (i < MAX_MSG_LENGTH) {
                msg = Msg(i);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this](Msg msg) { this->function1a(msg); });
                j++;
                counter++;
            }
            else {
                // End of inner and outer loop
                elapsed_time = get_current_time() - start_time;
            }
        }
    }
    
private:
    int i, j;
    Msg msg;
    int counter;
};
```

### p1420-coroutines-segmentation

The implementation of op1 now looks as follows:

```c++
class RemoteObject1Co
{
public:
    RemoteObject1Co(RemoteObjectImplCo& remoteObjImplCo)
        : m_remoteObjImplCo(remoteObjImplCo)
    {
    }

    async_task<Msg> op1(Msg msg)
    {
        // Write part
        Buffer writebuffer;
        printf("RemoteObject1Co::op1()\n");
        // Marshall msg into the buffer
        // (code not present)
        // Write the buffer in segments of size SEGMENT_LENGTH (onto the remote object)
        // until the whole buffer has been written (offset >= writebuffer.length())
        int buflength = writebuffer.length();
        for (int offset = 0; offset < buflength; offset += SEGMENT_LENGTH)
        {
            int bytestowrite = (buflength - offset) > SEGMENT_LENGTH ? SEGMENT_LENGTH : buflength - offset;
            printf("RemoteObject1Co::op1(): calling write_segment: offset = %d\n", offset);
            co_await m_remoteObjImplCo.start_write_segment(writebuffer.buffer(), offset, bytestowrite);
        }

        // Read part
        bool completed = false;
        Buffer readbuffer;
        Msg res;
        // Read the buffer in segments of size SEGMENT_LENGTH
        // until start_read_segment reports that the read is complete.
        for (int offset = 0; !completed; offset += SEGMENT_LENGTH)
        {
            printf("RemoteObject1Co::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await m_remoteObjImplCo.start_read_segment(readbuffer.buffer(), offset, SEGMENT_LENGTH);
        }
        // Unmarshall Msg from readbuffer
        // (code not present)
        // return the msg to the caller
        co_return res;
    }

protected:
    RemoteObjectImplCo& m_remoteObjImplCo;
};
```

The application class now looks as follows:

```c++
class Class01
{
public:
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::coroutine1()\n");
        start_time = get_current_time();
        for (int i = 0; i < MAX_MSG_LENGTH; i++)
        {
            printf("Class01::coroutine1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < NR_MSGS_TO_SEND; j++)
            {
                printf("Class01::coroutine1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_task<Msg> op1 = remoteObj1.op1(msg);
                Msg res = co_await op1;
                // Do something with msg
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};
```

## Comparison

This section contains a comparison of the advantages and disadvantages of the different styles illustrated above.

### Synchronous style

Advantages:

* Natural coding style, same style as if all function calls (method invocations) are local.

Disadvantages:

* Program is not reactive: during a RMI the calling function is blocked and the program cannot respond to other inputs:
This extends to all functions in the call tree: the program cannot react to other inputs during the RMI.

### Synchronous style with threads

Advantages:

* Program is reactive again, with only minimal changes to the original program.

Disadvantages:

* Overhead of thread creation (stack, scheduling).
* If functions share variables, race conditions are possible. 
Mutexes and atomic access may have to be used.
This approach may lead to sporadic errors (Heigenbugs) that are difficult to reproduce and to correct.


### Asynchronous style

Advantages:

* Program is reactive: program can respond to other inputs after the request to the server has been sent.

Disadvantages:

* Original function and its control flow is split into two or more functions, depending on the number of RMIs: there is one additional function per RMI.
This additional function handles the response of an RMI and may start other RMIs. All functions need to have a name.
* Use of lambda expressions at the application level makes the code more difficult to read.
* Artificial code for complex programs: e.g. it is difficult to recognize and implement nested loops.
* Longer and more complicated application level code.
* May be diffiult to distinguish "primary" functions (attached to input to the program) from
  "secondary" fuunctions that are attached to the response to RMI.

### Asynchronous style with local event loop

Advantages:

* After having sent a request to the remote server, the program can proceed with some tasks that
do not need the result of the remote method invocation. This can save some time, although usually the gain will be rather low.

Disadvantages:

* Still not possible to handle other events while being in the function1 call.

### Coroutines

Advantages:

* Natural style, very close to the original synchronous program.
* Program is reactive: program can respond to other inputs while the response of the remote server has not arrived.
* Before calling co_await, the program can do some work that does not need the response of the RMI.

Disadvantages;

* Some infrastructure code (coroutine layer) is needed to implement the coroutine alternative.

## Conclusions

Using coroutines we can stay close to the natural synchronous coding style of a program, 
while making the program reactive as in the asynchronous case.
