# Various stand-alone Boost applications

At the moment this folder contains only an example using timers in combination with coroutines.

Notice that op_timer1 (returned from start_timer) can be co_await-ed for several times because 
it has been instructed to reset itself.

    async_operation<void> op_timer1 = start_timer(timer1, 0);
    op_timer1.auto_reset(true);
    co_await op_timer1;
    print(PRI1, "--- timerTask01: after co_await op_timer1 --- 0\n");

    for (int i = 0; i < 3; i++)
    {
        async_operation<void> op_timer1a = start_timer(timer1, 1000);
        co_await op_timer1a;
        print(PRI1, "--- timerTask01: after co_await op_timer1a --- 1000\n");

        // ...
		
        async_operation<void> op_timer1e = start_timer(timer1, 5000);
        co_await op_timer1e;
        print(PRI1, "--- timerTask01: after co_await op_timer1e --- 5000\n");
    }
