'''
File: p1100a.py
Brief: Demonstrates lazy start

Author: Johan Vanslembrouck
'''

import asyncio

from printcl import printcl


async def completes_synchronously(i):
    return i


async def loop_synchronously(counter):
    v = 0
    for i in range(0, counter):
        ls = completes_synchronously(i)
        v += await ls
    printcl(f"loop_synchronously(): return {v}")
    return v


def main():
    printcl("main(): ls = loop_synchronously(10000)")
    ls = loop_synchronously(10000)
    printcl("main(): asyncio.run(ls)")
    v = asyncio.run(ls)
    printcl(f"main(): v = {v}")


if __name__ == "__main__":
    main()
