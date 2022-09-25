/** 
 *  Filename: p0010.cpp
 *  Description: Demonstrates the use of future, promise and thread
 *  
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: http://www.cplusplus.com/reference/future/async/
 */

#include <iostream>       // std::cout
#include <thread>
#include <future>         // std::async, std::future

// a non-optimized way of checking for prime numbers
bool is_prime (int x) {
    std::cout << "Calculating. Please, wait...\n";
    for (int i=2; i<x; ++i) 
        if (x%i==0) return false;
    return true;
}

void calculate_is_prime(std::promise<int> * promObj, int numbertotest)
{
    bool res = is_prime (numbertotest);
    promObj->set_value(res);
}
 
int main()
{
    std::promise<int> promiseObj;
    std::future<int> futureObj = promiseObj.get_future();
    std::thread th(calculate_is_prime, &promiseObj, 313222313);

    std::cout << "Checking whether 313222313 is prime.\n";
    // ...

    bool ret = futureObj.get();
    th.join();

    if (ret)
        std::cout << "It is prime!\n";
    else
        std::cout << "It is not prime.\n";

    return 0;
}

