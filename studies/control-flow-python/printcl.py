'''
File: printcl.py
Brief:

Author: Johan Vanslembrouck
'''

import datetime
import threading

threadList = []


def getthreadid():
    tid = threading.get_ident()
    try:
        idx = threadList.index(tid)
    except ValueError:
        idx = len(threadList)
        threadList.append(tid)
    result = "{:02d}".format(idx)
    return result


def printcl(txt):
    print(f"{getthreadid()}: {txt}")
