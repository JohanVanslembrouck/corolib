# Why use C++ coroutines for (distributed) applications?

This directory contains various examples that explain the advantage of C++ coroutines 
for writing (distributed) applications.

Consider a program name p1XYZ-purpose-of-the-program.cpp:

* X stands for programs with the same functionality for the same value of X
* Y = 0 is used for the synchronous version, Y = 1 is used for the asynchronous version and Y = 2 is used for the coroutine version
* Z = 0 is used for the base version. Variants are numbered 2, 4, etc.

No real remote method invocations (RMIs) are used in the examples.
Instead, the delay that may result of the invocation is simulated by means of a timer.

## Case 1: program with 1 RMI

### p1000-sync-1rmi.cpp

To be completed.

### p1010-async-1rmi.cpp

To be completed.

### p1020-coroutines-1rmi.cpp

To be completed.

## Case 2: program with callstack and 1 RMI

### p1100-sync-callstack-1rmi.cpp

To be completed.

### p1110-async-callstack-1rmi.cpp

To be completed.

### p1112-async-callstack-1rmi-queue-cs.cpp

To be completed.

### p1120-coroutines-callstack-1rmi.cpp

To be completed.

## Case 3: program with if-then-else and 3 RMIs

Function with 3 remote method invocations (RMIs) and an if-then-else. Depending on the result of the first RMI,
we enter the if-then or else case, where another remote method is invoked.

### p1200-sync-3rmis.cpp

The original code looks as follows:

```c++
struct Class01 {
    void function1() {
        printf("Class01::function1()\n");
        int ret1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
        // 1 Do stuff
        if (ret1 == gval1) {
            int ret2 = remoteObj2.op2(gin21, gin22, gout21);
            // 2 Do stuff
        }
        else {
            int ret3 = remoteObj3.op3(gin31, gout31, gout32);
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

### p1202-sync+thread-3rmis.cpp

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


### p1210-async-3rmis.cpp

```c++
struct Class01 {
    void function1() {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int out11, int out12, int ret1) {
        printf("Class01::function1a(%d, %d, %d)\n", out11, out12, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            remoteObj2.sendc_op2(gin21, gin22,
                [this](int out1, int ret1){ this->function1b(out1, ret1); });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(gin31, 
                [this](int out1, int out2, int ret1) { this->function1c(out1, out2, ret1); });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out21, int ret2) {
        printf("Class01::function1b()\n");
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out31, int out32, int ret3) {
        printf("Class01::function1c()\n");
        // 3b Do stuff that needs the result of the RMI
    }

    void function2() {
        printf("Class01::function2()\n");
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


### p1212-async-3rmis-local-event-loop.cpp

This example uses a local event loop that is placed close after the invocation of an asychronous variant
of the original two-way in-out RMI.

The local event loop can only handle the czllback function passed to the asynchronous function.

```c++
struct Class01 {
    void function1() {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(gin11, gin12,
            [this](int out1, int out2, int ret1) { this->callback1(out1, out2, ret1); });
        // 1a Do some stuff that doesn't need the result of the RMI
        eventQueue.run();
        // 1b Do stuff that needs the result of the RMI
        if (gret1 == gval1) {
            remoteObj2.sendc_op2(gin21, gin22, 
                [this](int out1, int ret1) { this->callback2(out1, ret1); });
            // 2a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 2b Do stuff that needs the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(gin31, 
                [this](int out1, int out2, int ret1) { this->callback3(out1, out2, ret1); });
            // 3a Do some stuff that doesn't need the result of the RMI
            eventQueue.run();
            // 3b Do stuff that needs the result of the RMI
        }
    }

    void callback1(int out11, int out12, int ret1) { 
        printf("Class01::callback1(%d, %d, %d)\n", out11, out12, ret1);
        // copy to local variables
        gret1 = ret1;
    }
    void callback2(int out21, int ret2) { 
        printf("Class01::callback2(%d, %d)\n", out21, ret2);
        // copy to local variables
    }
    void callback3(int out31, int out32, int ret3) {
        printf("Class01::callback3(%d, %d, %d)\n", out31, out32, ret3);
        // copy to local variables
    }
    void function2() { 
        printf("Class01::function2()\n");
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


### p1220-coroutines-3rmis.cpp


```c++
struct Class01
{
	async_task<void> coroutine1()
	{
		int ret1 = co_await remoteObj1.op1(gin11, gin12, gout11, gout12);
		// 1 Do stuff
		if (ret1 == gval1) {
			int ret2 = co_await remoteObj2.op2(gin21, gin22, gout21);
			// 2 Do stuff
		}
		else {
			int ret3 = co_await remoteObj3.op3(gin31, gout31, gout32);
			// 3 Do stuff
		}
	}
	
	async_task<void> coroutine1a()
	{
		async_task<int> op1 = remoteObj1.op1(gin11, gin12, gout11, gout12);
		// 1a Do some stuff that doesn't need the result of the RMI
		int ret1 = co_await op1;
		// 1b Do stuff that needs the result of the RMI
		if (ret1 == gval1) {
			async_task<int> op2 = remoteObj2.op2(gin21, gin22, gout21);
			// 2a Do some stuff that doesn't need the result of the RMI
			int ret2 = co_await op2;
			// 2b Do stuff that needs the result of the RMI
		}
		else {
			async_task<int> op3 = remoteObj3.op3(gin31, gout31, gout32);
			// 3a Do some stuff that doesn't need the result of the RMI
			int ret3 = co_await op3;
			// 3b Do stuff that needs the result of the RMI
		}
	}
};
```

Advantages:

* Natural style, very close to the original program (in this case p1200-sync-3rmis.cpp).
* Program can respond to other inputs while the response of the remote server has not arrived.
* Program can do some work that does not need response of the remote method invocation.

Disadvantages;

* All output arguments and the return value have to be packed into a structure that will be returned by co_await.
* Rather some infrastructure code needed to implement the coroutine alternative.


## Case 4: program with nested for loop

### p1300-sync-nested-loop.cpp


```c++
struct Class01 {
    void function1() {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class04::function1(): i = %d\n", i);
            Msg msg(i);
            for (int j = 0; j < nr_msgs_to_send; j++) {
                printf("Class04::function1(): i = %d, j = %d, counter = %d\n", i, j, counter++);
                int ret1 = remoteObj1.op1(msg);
            }
        }
        elapsed_time = get_current_time() - start_time;
    }
    void function2() { 
        printf("Class01::function2()\n");
    }
};
```

Advantages:

* Natural style.

Disadvantages

* Program is not reactive.

### p1310-async-nested-loop.cpp


```c++
struct Class01 {
    int i = 0, j = 0;
    Msg msg;
    int counter = 0;

    void function1() {
        printf("Class01::function1(): counter = %d\n", counter);
        start_time = get_current_time();
        msg = Msg(0);
        remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
    }

    void function1a() {
        printf("Class01::function1a(): counter = %d\n", counter);
        if (j < nr_msgs_to_send) {
            printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
            remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
            j++;
            counter++;
        }
        else {
            j = 0;
            i++;
            if (i < max_msg_length) {
                msg = Msg(i);
                printf("Class01::function1a(): i = %d, j = %d, counter = %d\n", i, j, counter);
                remoteObj1.sendc_op1(msg, [this]() { this->function1a(); });
                j++;
                counter++;
            }
            else
                elapsed_time = get_current_time() - start_time;
        }
    }

    void function2() {
        printf("Class01::function2()\n");
    }
};
```

Advantages:

* TBC

Disadvantages:

* Artificial code: Especially it is difficult to recognize and write the nested iteration.

### p1320-coroutines-nested-loop.cpp


```c++
struct Class01
{
    async_task<void> coroutine1()
    {
        int counter = 0;
        printf("Class01::function1()\n");
        start_time = get_current_time();
        for (int i = 0; i < max_msg_length; i++) {
            printf("Class01::function1(): i = %d\n", i);
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

## Case 5: Segmentation: adding two loops at the infrastructure level

The request and response to and from the remote server cannot be transmitted in a single packet.
Instead, it has to be split into small packets that each have to acknowledged before the next packet can be sent.

### p1400-sync-segmentation.cpp


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
        return gret1;
    }
};
```


## p1410-async-segmentation.cpp


```c++
struct RemoteObject3 {
    int offset = 0;
    Buffer buf;
    bool completed = false;
    Buffer buf2;
    lambda_3int_t lambda;

    void sendc_op1(int in11, int in12, lambda_3int_t op1cb) {
        printf("RemoteObject3::sendc_op1(): calling write_segment\n");
        lambda = op1cb;
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
            lambda(gout11, gout12, gret1);
        }
    }

    void callback(int out11, int out12, int ret1) {
        printf("RemoteObject3::callback()\n");
    }
};
```


## p1420-coroutines-segmentation.cpp


```c++
struct RemoteObject2 {
    async_task<Msg> op1(Msg msg) {
        Buffer buf;
        Msg res;
        printf("RemoteObject2::op1()\n");
        // Marshall msg into buf
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
        // Unmarshall Msg from buf2
        co_return res;
    }
};
```

## Conclusions

Using coroutines we can stay close to the natural synchronous coding style of a program, 
while making the program reactive again.
