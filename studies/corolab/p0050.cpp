/** 
 *  Filename: p0050.cpp
 *  Description: Demonstates the use of thread (without condition variable or mutex)
 *  
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon: http://www.cplusplus.com/reference/future/async/
 *              https://thispointer.com//c11-multithreading-part-7-condition-variables-explained/
 */
 
#include <iostream>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

using namespace std::placeholders;

// a non-optimized way of checking for prime numbers
bool is_prime (int x) {
    std::cout << "Calculating. Please, wait...\n";
    for (int i=2; i<x; ++i) 
        if (x%i==0) return false;
    return true;
}

class Application
{
    bool m_bCalculated;
    bool m_iValue;

public:
    Application()
    {
        m_bCalculated = false;
    }
  
    void calculate_is_prime(int numbertotest)
    {
        m_iValue = is_prime(numbertotest);

        // Set the flag to true
        m_bCalculated = true;
    }
  
    bool getValue()
    {
        // Check if flag is set to true or not
        while(m_bCalculated != true)
        {
            //sleep for 100 milliseconds
            std::cout << "Sleeping for 100 ms.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return m_iValue;
     }
};

int main()
{
    Application app;
    std::thread th(&Application::calculate_is_prime, &app, 313222313);

    std::cout << "Checking whether 313222313 is prime.\n";
    // ...

    bool ret = app.getValue();
    th.join();

    if (ret)
        std::cout << "It is prime!\n";
    else
        std::cout << "It is not prime.\n";

    return 0;
}

