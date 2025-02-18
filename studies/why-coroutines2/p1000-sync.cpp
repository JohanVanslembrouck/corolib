/**
 * @file p1000-sync.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "corolib/print.h"

#include "sync.h"

using namespace corolib;

void synchronous(int st)
{
    int i = st;
    print(PRI1, "synchronous: begin\n");

    print(PRI1, "synchronous: %d: create\n", st);
    i = create(i + 1);
    assert(i == st + 1);

	print(PRI1, "synchronous: %d: create complete: open\n", i);
    i = open(i + 1);
    assert(i == st + 2);

    print(PRI1, "synchronous: %d: open complete: write\n", i);
    i = write(i + 1);
    assert(i == st + 3);

	print(PRI1, "synchronous: %d: write complete: read\n", i);
	i = read(i + 1);
    assert(i == st + 4);

	print(PRI1, "synchronous: %d: read complete: close\n", i);
    i = close(i + 1);
    assert(i == st + 5);

	print(PRI1, "synchronous: %d: close complete: remove\n", i);
    i = remove(i + 1);
    assert(i == st + 6);

	print(PRI1, "synchronous: %d: remove complete: done\n", i);
    print(PRI1, "synchronous: end\n");
}

int main() {
    print(PRI1, "main: begin\n");
	synchronous(10);
    synchronous(20);
    print(PRI1, "main: end\n");
	return 0;
}
