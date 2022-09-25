/**
 *  Filename: p0200.cpp
 *  Description:
 *  Shows 3 ways to iterate over prime numbers until a maximum is reached
 *  and to print them.
 *  Uses only C++11 features.
 *  All three approaches have their drawbacks.
 *
 *  Tested with Visual Studio 2019 and g++ 4.8.5 on Linux.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *  Based upon:
 */

#include <iostream>

 // a non-optimized way of checking for prime numbers
bool is_prime(int x) {
    for (int i = 2; i < x; ++i)
        if (x % i == 0) return false;
    return true;
}

/**
 * Approach 1: printing is embedded in the primes function.
 * Another function is needed when we want to do
 * something else than printing, or print them in anothher way.
 *
 */
void primes1(std::size_t max) {
    for (unsigned int number = 2; number < max; ++number) {
        if (is_prime(number)) {
            std::cout << number << " is prime" << std::endl;
        }
    }
}

/**
 * Approach 2: passing a callback to the primes function.
 *
 *
 */
template<typename L>
void primes2(std::size_t max, L found) {
    for (unsigned int number = 2; number < max; ++number) {
        if (is_prime(number))
            found(number);
    }
}

/**
 * Approach 3: using an interator.
 * This iterator is specific for primes, so we need another
 * iterator to iterate over anything else.
 *
 */
class prime_iterator {
    unsigned int number = 2;
public:
    int operator * () const {
        return number;
    }

    prime_iterator& operator ++ () {
        for (++number; !is_prime(number); ++number);
        return *this;
    }
};

int main()
{
    primes1(30);

    std::cout << std::endl;

    primes2(30,
        [](int n) {
            std::cout << n << " is prime" << std::endl;
        }
    );

    std::cout << std::endl;

    for (prime_iterator p; *p < 30; ++p) {
        std::cout << *p << " is prime" << std::endl;
    }

    return 0;
}
