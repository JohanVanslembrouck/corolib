# Why use C++ coroutines for (distributed) applications?

This directory contains various examples that explain the advantage of C++ coroutines 
for writing (distributed) applications.

I will consider two types of programs: programs not using coroutines, named co-lessXX.cpp, and
programs using coroutines, called co-fullXX.cpp.

First I discuss the advantages and disadvantages the original program and of variants of that program that do not use coroutines.
Then I show how we can stay close to the natural style of the first coroutine-less program while solving
the disadvantages of this program and its variants using coroutines, while trying to avoid or limit disadvantages.

No real remote method invocations (RMIs) are used in the examples.
Instead, the delay that may result of the invocation is simulated by means of a timer.

## Case 1: program with if-then-else

Function with 3 remote method invocations (RMIs) and an if-then-else. Depending on the result of the first RMI,
we enter the if-then or else case, where another remote method is invoked.

### co-less01.cpp

The original code looks as follows:

```c++
struct Class01 {
    void function1() {
        printf("Class01::function1()\n");
        ret1 = remoteObj1.op1(in11, in12, out11, out12);
        // 1 Do stuff
        if (ret1 == val1) {
            ret2 = remoteObj2.op2(in21, in22, out21);
            // 2 Do stuff
        }
        else {
            ret3 = remoteObj3.op3(in31, out31, out32);
            // 3 Do stuff
        }
    }
    void function2() { }
};
```

The method invocations look as local method invocations, but they can be remote method invocations as well.
The code for this remote invocation can be completely generated from an interface definition, e.g. written using CORBA's IDL.

Advantages:

* Natural coding style, same style as if all function calls (method invocations) are local.

Disadvantages:

* Program is not reactive: during a remote method invocation the calling function is blocked and the program cannot respond to other inputs:
This extends to the call of function1(): the program cannot react to other inputs while function1() is called.

The following code shows how function1 is called when the program receives event1 or event2.
Normally a different function will be "connected" to every event.

```c++
int main() {
    printf("main();\n");
    connect(event1, []() { class01.function1(); });
    connect(event2, []() { class01.function1(); });
    eventQueue.run();
    return 0;
}
```
The name "connect" has been inspired by the connect mechanism used by Qt to connect signals to slots.

### co-less01th.cpp

It is possible to make the program reactive by running every function on its own thread.
The implementation of function1 does not have to be changed. This can be done as follows:

```c++
int main() {
    printf("main();\n");
    connect(event1, []() { std::thread th(&Class01::function1, &class01); th.join(); });
    connect(event2, []() { std::thread th(&Class01::function1, &class01); th.join(); });
    eventQueue.run();
    return 0;
}
```

Advantages:

* Program is reactive again, with only minimal changes.

Disadvantages:

* Overhead of thread creation (stack, scheduling).
* If functions share variables, then race conditions are possible. 
Mutexes and atomic access may have to be used.
This approach may lead to sporadic errors (Heigenbugs) that are difficult to reproduce and to correct.

### co-less02.cpp

This example is an intermediate step towards co-less03.cpp.
This example uses a local event loop that is placed close after the invocation of an asychronous variant
of the original two-way in-out RMI.

The local event loop can only handle the czllback function passed to the asynchronous function.

```c++
struct Class02 {
    void function1() {
        printf("ClassA2::function1()\n");
        remoteObj1.sendc_op1(in11, in12,
            [this](int out1, int out2, int ret1) { this->callback1(out1, out2, ret1); });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            remoteObj2.sendc_op2(in21, in22, 
                [this](int out1, int ret1) { this->callback2(out1, ret1); });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in31, 
                [this](int out1, int out2, int ret1) { this->callback3(out1, out2, ret1); });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

    void callback1(int out11, int out12, int ret1) { 
        printf("ClassA2::callback1(%d, %d, %d)\n", out11, out12, ret1);
        // copy to local variables 
    }
    void callback2(int out21, int ret2) { 
        printf("ClassA2::callback2(%d, %d)\n", out21, ret2);
        // copy to local variables
    }
    void callback3(int out31, int out32, int ret3) {
        printf("ClassA2::callback3(%d, %d, %d)\n", out31, out32, ret3);
        // copy to local variables
    }
    void function2() { 
        printf("ClassA2::function2()\n");
    }
};
```

Advantages:

* After having sent a request to the remote server, the program can proceed with some tasks that
do not need the result of the remote method invocation. This can save some time, although usually the gain will be rather low.

Disadvantages:

* Still not possible to handle other events while being in the function1 call.
* Use of lambda expressions at the application level makes the code more difficult to read.
* Longer complicated application level code.

### co-less03.cpp


```c++
struct Class03 {
    void function1() {
        printf("Class03::function1()\n");
        remoteObj1.sendc_op1(in11, in12, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int out11, int out12, int ret1) {
        printf("Class03::function1a(%d, %d, %d)\n", out11, out12, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            remoteObj2.sendc_op2(in21, in22,
                [this](int out1, int ret1){ this->function1b(out1, ret1); });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in31, 
                [this](int out1, int out2, int ret1) { this->function1c(out1, out2, ret1); });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out21, int ret2) {
        printf("Class03::function1b()\n");
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out31, int out32, int ret3) {
        printf("Class03::function1c()\n");
        // 3b Do stuff that needs the result of the RMI
    }

    void function2() {
        printf("Class03::function2()\n");
    }
};
```

Advantages:

* Program can respond to other inputs after the request to the server has been sent.

Disadvantages:

* Original function and its control flow is split into a number of functions (in this example four).
* Unnatural style
* May be diffiult to distinguish "primary" functions (attached to input to the programs) from
"secondary" fuunctions that are attached to the response to a remote method invocation.

### co-full01.cpp


```c++
class Class01
{
public:
    async_task<void> coroutine1()
    {
        printf("Class01::coroutine1()\n");
        async_operation<op1_ret_t> op1 = remoteObj1.start_op1(in11, in12);
        // 1a Do some stuff that doesn't need the result of the RMI
        co_await op1;
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == val1) {
            async_operation<op2_ret_t> op2 = remoteObj2.start_op2(in21, in22);
            // 2a Do some stuff that doesn't need the result of the RMI
            co_await op2;
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            async_operation<op1_ret_t> op3 = remoteObj3.start_op3(in31);
            // 3a Do some stuff that doesn't need the result of the RMI
            co_await op3;
            // 3b Do stuff that needs the result of the RMI
        }
    }
};
```

Advantages:

* Natural style, very close to the original program (in this case co-less01.cpp).
* Program can respond to other inputs while the response of the remote server has not arrived.
* Program can do some work that does not need response of the remote method invocation.

Disadvantages;

* All output arguments and the return value have to be packed into a structure that will be returned by co_await.
* Rather some infrastructure code needed to implement the coroutine alternative.


## Case 2: program with nested for loop

### co-less04.cpp


```c++
struct Class04 {
    void function1() {
        int counter = 0;
        printf("Class04::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class04::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class04::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                ret1 = remoteObj1.op1(msg);
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
    void function2() { 
        printf("Class04::function2()\n");
    }
};
```

Advantages:

* Natural style.

Disadvantages

* Program is not reactive.

### co-less05.cpp


```c++
struct Class05 {
    int i, j = 0;
    Msg msg;
    int counter = 0;

    void function1() {
        printf("Class05::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a() {
        printf("Class05::function1a(): counter = %d\n", counter);
        if (j < nr_msgs_to_send) {
            printf("Class05::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < max_msg_length) {
                msg = Msg(i);
                printf("Class05::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
    }

    void function2() {
        printf("Class05::function2()\n");
    }
};
```

Advantages:

* TBC

Disadvantages:

* Artificial code: Especially it is difficult to recognize and write the nested iteration.

### co-full02.cpp


```c++
class Class02
{
public:
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class02::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class02::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class02::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                async_operation<int> op1 = remoteObj1.start_op1(msg);
                int i = co_await op1;
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
};
```

Advantages:

* Natural style.
* Program is reactive.

Disadvantages:

* TBC

## Case 3: Adding two loops at the infrastructure level

The request and response to and from the remote server cannot be transmitted in a single packet.
Instead, it has to be split into small packets that each have to acknowledged before the next packet can be sent.

### co-less06.cpp


```c++
struct RemoteObject2 {
    int op1(int in11, int in12, int& out11, int& out12) {
        Buffer buf;
        printf("RemoteObject2::op1()\n");
        // Marshall in11 and in12 into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject2::op1(): calling write_segment\n");
            remoteObjImpl.write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject2::op1(): calling read_segment\n");
            completed = remoteObjImpl.read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall out11, out12 and ret1 from buf2
        return ret1;
    }
};
```


## co-less07.cpp


```c++
struct RemoteObject3 {
    int offset = 0;
    Buffer buf;
    bool completed = false;
    Buffer buf2;
    lambda1 l;

    void sendc_op1(int in11, int in12, lambda1 op1cb) {
        printf("RemoteObject3::sendc_op1(): calling write_segment\n");
        l = op1cb;
        // Marshall in11 and in12 into buf
        remoteObjImpl.sendc_write_segment(buf.buffer(), offset,
            [this]() { this->op1a(); });
    }

    void op1a() {
        printf("RemoteObject3::op1a()\n");
        if (offset < buf.length()) {
            printf("RemoteObject3::op1a(): calling sendc_write_segment\n");
            remoteObjImpl.sendc_write_segment(buf.buffer(), offset, 
                [this]() { this->op1a(); });
            offset += segment_length;
        }
        else {
            offset = 0;
            printf("RemoteObject3::op1a(): calling sendc_read_segment\n");
            remoteObjImpl.sendc_read_segment(buf2.buffer(), offset, segment_length, 
                                                            [this]() { this->op1b(); });
        }
    }

    void op1b() {
        printf("RemoteObject3::sendc_op1b(): calling sendc_read_segment\n");
        if (offset < segment_length) {
            remoteObjImpl.sendc_read_segment(buf2.buffer(), offset, segment_length, 
                                                            [this]() { this->op1b(); });
            offset += segment_length;
        }
        else {
            // Unmarshall out11, out12 and ret1 from buf2
            l(out11, out12, ret1);
        }
    }

    void callback(int out11, int out12, int ret1) {
        printf("RemoteObject3::callback()\n");
    }
};
```



## co-less08.cpp

Not yet implemented.


## co-full03.cpp


```c++
struct RemoteObject2 {
    async_task<op1_ret_t> op1(int in11, int in12) {
        Buffer buf;
        op1_ret_t res{ 1, 2, 3 };
        printf("RemoteObject2::op1()\n");
        // Marshall in11 and in12 into buf
        for (int offset = 0; offset < buf.length(); offset += segment_length) {
            printf("RemoteObject2::op1(): calling write_segment: offset = %d\n", offset);
            co_await remoteObjImpl.start_write_segment(buf.buffer(), offset);
        }
        bool completed = false;
        Buffer buf2;
        for (int offset = 0; !completed; offset += segment_length) {
            printf("RemoteObject2::op1(): calling read_segment: offset = %d\n", offset);
            completed = co_await remoteObjImpl.start_read_segment(buf2.buffer(), offset, segment_length);
        }
        // Unmarshall out11, out12 and ret1 from buf2
        co_return res;
    }
};
```

## Conclusions

Using coroutines we can stay close to the natural synchronous coding style of a program, 
while making the program reactive again.

