'''
File: p2020.py
Brief: Demonstrates lazy start

Author: Johan Vanslembrouck
'''

import asyncio

from printcl import printcl


async def foo():
    printcl("foo(): await asyncio.sleep(1)")
    await asyncio.sleep(1)
    printcl("foo(): return 1")
    return 1


async def bar():
    printcl("bar(): f = foo()")
    f = foo()
    printcl("bar(): v = await f")
    v = await f
    printcl(f"bar(): return {v + 1}")
    return v + 1


def main():
    printcl("main(): b = bar()")
    b = bar()
    printcl("main(): asyncio.run(b)")
    v = asyncio.run(b)
    printcl(f"main(): v = {v}")


if __name__ == "__main__":
    main()
