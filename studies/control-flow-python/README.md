# Study: Control flow in Python

This directory contains the Python equivalent scripts of the examples
in [control-flow](../control-flow) and [control-flow-cs](../control-flow-cs).

The following is the output of the four examples:

```
C:\...\corolib\studies\control-flow-python>python p1100.py
00: main(): ls = loop_synchronously(4)
00: main(): asyncio.run(ls)
00: loop_synchronously(4)
00: loop_synchronously(): cs = completes_synchronously(0)
00: loop_synchronously(): v += await cs
00: completes_synchronously(0): return 0
00: loop_synchronously(): cs = completes_synchronously(1)
00: loop_synchronously(): v += await cs
00: completes_synchronously(1): return 1
00: loop_synchronously(): cs = completes_synchronously(2)
00: loop_synchronously(): v += await cs
00: completes_synchronously(2): return 2
00: loop_synchronously(): cs = completes_synchronously(3)
00: loop_synchronously(): v += await cs
00: completes_synchronously(3): return 3
00: loop_synchronously(): return 6
00: main(): v = 6

C:\...\corolib\studies\control-flow-python>python p1100a.py
00: main(): ls = loop_synchronously(10000)
00: main(): asyncio.run(ls)
00: loop_synchronously(): return 49995000
00: main(): v = 49995000

C:\...\corolib\studies\control-flow-python>python p2010-sc.py
00: main(): b = bar()
00: main(): asyncio.run(b)
00: bar(): f = foo()
00: bar(): v = await f
00: foo(): return 1
00: bar(): return 2
00: main(): v = 2

C:\...\corolib\studies\control-flow-python>python p2020.py
00: main(): b = bar()
00: main(): asyncio.run(b)
00: bar(): f = foo()
00: bar(): v = await f
00: foo(): await asyncio.sleep(1)
00: foo(): return 1
00: bar(): return 2
00: main(): v = 2
PS C:\...\corolib\studies\control-flow-python>
```

The examples show that Python uses laay start: a coroutine must be awaited before we enter the body of the coroutine.
It also shows that all coroutine code runs on a single thread.

Consider the following code extract from p1100.py.
The numbers indicate the order in which statements are executed.

```
async def completes_synchronously(i):
    printcl(f"completes_synchronously({i}): return {i}")                        # 3
    return i                                                                    # 3                   

async def loop_synchronously(counter):
    printcl(f"loop_synchronously({counter})")
    v = 0
    for i in range(0, counter):
        printcl(f"loop_synchronously(): cs = completes_synchronously({i})")     # 1
        cs = completes_synchronously(i)                                         # 1
        printcl("loop_synchronously(): v += await cs")                          # 2
        v += await cs                                                           # 2
    printcl(f"loop_synchronously(): return {v}")
```

Consider the following code extract from p2010-sc.py.
The numbers indicate the order in which statements are executed.

```
async def foo():
    printcl(f"foo(): return 1")                         # 3
    return 1                                            # 3
  
async def bar():
    printcl(f"bar(): f = foo()")                        # 1
    f = foo()                                           # 1
    printcl("bar(): v = await f")                       # 2
    v = await f                                         # 2
    printcl(f"bar(): return {v + 1}")
    return v + 1
```
