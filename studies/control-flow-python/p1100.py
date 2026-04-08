'''
File: p1100.py
Brief: Demonstrates lazy start

Author: Johan Vanslembrouck
'''

import asyncio

from printcl import printcl


async def completes_synchronously(i):
    printcl(f"completes_synchronously({i}): return {i}")
    return i


async def loop_synchronously(counter):
    printcl(f"loop_synchronously({counter})")
    v = 0
    for i in range(0, counter):
        printcl(f"loop_synchronously(): cs = completes_synchronously({i})")
        cs = completes_synchronously(i)
        printcl("loop_synchronously(): v += await cs")
        v += await cs
    printcl(f"loop_synchronously(): return {v}")
    return v


def main():
    printcl("main(): ls = loop_synchronously(4)")
    ls = loop_synchronously(4)
    printcl("main(): asyncio.run(ls)")
    v = asyncio.run(ls)
    printcl(f"main(): v = {v}")


if __name__ == "__main__":
    main()
