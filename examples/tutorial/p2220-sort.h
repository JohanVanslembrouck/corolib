/**
 * @file p2220-sort.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P2220_SORT_H_
#define _P2220_SORT_H_

#include <vector>

void serial_sort(std::vector<int>& v);
void concurrent_sort(std::vector<int>& v);

#endif
