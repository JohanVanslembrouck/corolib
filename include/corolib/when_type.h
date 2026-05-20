/**
 * @file when_type.h
 * @brief concept WHEN_TYPE replaces async_base that is (was) used
 * as base class for the classes defined in async_task.h and async_operation.h.
 * (See async_base.h for more information.)
 * concept WHEN_TYPE is used in the definition if when_all and when_any.
 * 
 * concept WHEN_TYPE is based on 
 * Guideline 7: Understand the Similarities Between Base Classes and Concepts
 * in the book C++ Software Design - Design Principles and Patterns for High-Quality Software
 * by Klaus Iglberger
 * https://www.oreilly.com/library/view/c-software-design/9781098113155/
 *
 * @author Johan Vanslembrouck
 */

#ifndef _WHEN_TYPE_H_
#define _WHEN_TYPE_H_

#include "when_all_counter.h"
#include "when_any_one.h"

namespace corolib
{
    template<typename TYPE>
    concept WHEN_TYPE =
        requires (TYPE t, when_all_counter* ctr, when_any_one* waitany) {
        t.is_ready();
        t.setCounter(ctr);
        t.setWaitAny(waitany);
        t.start();
    };
}

#endif
