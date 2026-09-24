/**
 * @file p2200-sort.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P2200_SORT_H_
#define _P2200_SORT_H_

#include "p2200.h"

void sortVector(std::vector<int>& values);
bool sortRandomNumberVectorSerial(int size);

async_task<void> sortVector(Sorter& sorter, std::vector<int>& values);
async_task<void> sortVector_lso(Sorter& sorter, std::vector<int>& values);  // lso = lazy-start operation

#endif
