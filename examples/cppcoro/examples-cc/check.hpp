/**
* @file check.hpp
* @brief
*
* @author Johan Vanslembrouck
*/

#pragma once

#include <iostream>

void CHECK(bool x)
{
    if (!x) std::cout << "CHECK: error\n";
}

void FAIL(const char* str)
{
    std::cout << "FAIL: " << str << "\n";
}
