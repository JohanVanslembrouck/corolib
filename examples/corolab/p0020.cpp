/** 
 *  Filename: p0020.cpp
 *  Description: Demonstrates the use of future and async
 *  
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com)
 *  Based upon: http://www.cplusplus.com/reference/future/async/
 */

#include <iostream>       // std::cout
#include <future>         // std::async, std::future

// a non-optimized way of checking for prime numbers
bool is_prime (int x) {
    std::cout << "Calculating. Please, wait...\n";
    for (int i=2; i<x; ++i) 
        if (x%i==0) return false;
    return true;
}

int main ()
{
    // call is_prime(313222313) asynchronously:
    std::future<bool> futureObj = std::async (is_prime,313222313);

    std::cout << "Checking whether 313222313 is prime.\n";
    // ...

    bool ret = futureObj.get();      // waiting for is_prime to return

    if (ret)
        std::cout << "It is prime!\n";
    else
        std::cout << "It is not prime.\n";

    return 0;
}
