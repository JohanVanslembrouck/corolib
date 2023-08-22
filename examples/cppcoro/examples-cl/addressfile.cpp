/**
* @file addressfile.cpp
* @brief
*
* @author Johan Vanslembrouck(johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
*/

#include <iostream>
#include <fstream>

#include "addressfile.hpp"

using namespace cppcoro;
using namespace cppcoro::net;

void saveServerAddress(ip_endpoint serverAddress)
{
    std::string str = serverAddress.to_string();
    std::cout << "serverAddress = " << str << std::endl;

    if (std::ofstream output{ "serverAddress.txt", std::ios::out })
    {
        output << str << std::endl;
    }
    else
    {
        std::cerr << "File serverAddress.txt could not be opened\n";
        std::exit(EXIT_FAILURE);
    }
}

std::string readServerAddress()
{
    std::string serverAddressStr{};

    if (std::ifstream input{ "serverAddress.txt", std::ios::in })
    {
        while (input >> serverAddressStr)
        {
            std::cout << serverAddressStr << std::endl;
        }
    }
    else
    {
        std::cerr << "File serverAddress.txt could not be opened\n";
        std::exit(EXIT_FAILURE);
    }

    return serverAddressStr;
}
