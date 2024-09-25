/** 
 *  Filename: p0030.cpp
 *  Description: Demonstrates the use of condition variable + mutex and thread
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
    std::mutex m_mutex;
    std::condition_variable m_condVar;
    bool m_bCalculated;
    bool m_iValue;

public:
    Application()
    {
        m_bCalculated = false;
    }
  
    void calculate_is_prime(int numbertotest)
    {
        // Lock The Data structure
        std::lock_guard<std::mutex> guard(m_mutex);

        m_iValue = is_prime(numbertotest);

        // Set the flag to true
        m_bCalculated = true;
        // Notify the condition variable
        m_condVar.notify_one();
    }
  
    bool isDataCalculated()
    {
        return m_bCalculated;
    }
  
    bool getValue()
    {
        // Acquire the lock
        std::unique_lock<std::mutex> mlock(m_mutex);
        // Start waiting for the Condition Variable to get signaled
        // Wait() will internally release the lock and make the thread to block
        // As soon as condition variable get signaled, resume the thread and
        // again acquire the lock. Then check if condition is met or not
        // If condition is met then continue else again go in wait.
        m_condVar.wait(mlock, std::bind(&Application::isDataCalculated, this));
        return m_iValue;;
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

