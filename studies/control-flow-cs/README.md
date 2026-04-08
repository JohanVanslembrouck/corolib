# Study: Control flow in C#

This directory contains the C# equivalent programs of the examples
in [control-flow](../control-flow) and [control-flow-python](../control-flow-python).

The following is the output of the four examples:

```
C:\...\corolib\studies\control-flow-cs>.\p1100\bin\Debug\net8.0\p1100.exe
01: Main(): Task<int> ls = loop_synchronously(4);
01: loop_synchronously(4)
01: loop_synchronously(): Task<int> cs = completes_synchronously(0);
01: completes_synchronously(0): return 0;
01: loop_synchronously(): v += await cs;
01: loop_synchronously(): v = 0
01: loop_synchronously(): Task<int> cs = completes_synchronously(1);
01: completes_synchronously(1): return 1;
01: loop_synchronously(): v += await cs;
01: loop_synchronously(): v = 1
01: loop_synchronously(): Task<int> cs = completes_synchronously(2);
01: completes_synchronously(2): return 2;
01: loop_synchronously(): v += await cs;
01: loop_synchronously(): v = 3
01: loop_synchronously(): Task<int> cs = completes_synchronously(3);
01: completes_synchronously(3): return 3;
01: loop_synchronously(): v += await cs;
01: loop_synchronously(): v = 6
01: loop_synchronously(): return 6;
01: Main(): ls.Wait();
01: Main(): int v = b.Result;
01: Main(): v = 6

C:\...\corolib\studies\control-flow-cs>.\p1100a\bin\Debug\net8.0\p1100a.exe
01: Main(): Task<int> ls = loop_synchronously(10000);
01: loop_synchronously() return 49995000;
01: Main(): ls.Wait();
01: Main(): int v = ls.Result;
01: Main(): v = 49995000

C:\...\corolib\studies\control-flow-cs>.\p2010-sc\bin\Debug\net8.0\p2010-sc.exe
01: Main(): Task b = bar();
01: bar(): Task<int> f = foo();
01: foo(): return 1;
01: bar(): int v = await f;
01: bar(): return 2;
01: Main(): b.Wait();
01: Main(): int v = b.Result;
01: Main(): v = 2

C:\...\corolib\studies\control-flow-cs>.\p2020\bin\Debug\net8.0\p2020.exe
01: Main(): Task b = bar();
01: bar(): Task<int> f = foo();
01: foo() await Task.Delay(1000);
01: bar(): int v = await f;
01: Main(): b.Wait();
05: foo(): return 1;
05: bar(): return 2;
01: Main(): int v = b.Result;
01: Main(): v = 2
PS C:\...\corolib\studies\control-flow-cs>
```

The examples show that C# uses laay start: we enter the body of a coroutine when the coroutine is called.
It also shows that continuation runs on a dedicates thread in p2020.exe.

Consider the following code extract from p1100.cs.
The numbers indicate the order in which statements are executed.

```
private static async Task<int> completes_synchronously(int i)
{
    printcl($"completes_synchronously({i}): return {i};");      // 2
    return i;                                                   // 2
}

private static async Task<int> loop_synchronously(int counter)
{
    printcl($"loop_synchronously({counter})");
    int v = 0;
    for (int i = 0; i < counter; ++i)
    {
        printcl($"loop_synchronously(): Task<int> cs = completes_synchronously({i});");     // 1
        Task<int> cs = completes_synchronously(i);                                          // 1
        printcl("loop_synchronously(): v += await cs;");        // 3
        v += await cs;                                          // 3
        printcl($"loop_synchronously(): v = {v}");
    }
    printcl($"loop_synchronously(): return {v};");
    return v;
}
```

Consider the following code extract from p2010-sc.cs.
The numbers indicate the order in which statements are executed.

```
private static async Task<int> foo()
{
    printcl("foo(): return 1;");                // 2
    return 1;                                   // 2
}

private static async Task<int> bar()
{
    printcl("bar(): Task<int> f = foo();");     // 1
    Task<int> f = foo();                        // 1
    printcl("bar(): int v = await f;");         // 3
    int v = await f;                            // 3
    printcl($"bar(): return {v + 1};");
    return v + 1;
}
```
