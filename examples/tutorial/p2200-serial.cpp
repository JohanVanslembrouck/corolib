/**
 * @file p2200-serial.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include "p2200-sort.h"

int main() 
{
   for (int i = 1; i <= 3; ++i)
   {
       print(PRI1, "main(): bool res = sortRandomNumberVectorSerial(%d * 10'000'000);\n", i);
       bool res = sortRandomNumberVectorSerial(i * 10'000'000);
       print(PRI1, "main(): res = %d\n", res);
   }

   return 0;
}
