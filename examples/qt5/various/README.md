# Various stand-alone Qt applications

At the moment this folder contains only examples using timers in combination with coroutines.

Notice that op_timer1 (returned from start_timer) can be co_await-ed for several times because 
it has been instructed to reset itself:

```c++
    async_operation<void> op_timer1 = start_timer(timer1, 0);
    op_timer1.auto_reset(true);

    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 0\n");

    for (int i = 0; i < 3; i++)
    {
        timer1.start(1000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 1000\n");

        // ...
		
        timer1.start(5000);
        co_await op_timer1;
        print(PRI1, "--- timerTask01: after co_await op_timer1 --- 5000\n");
    }
```
