/**
 * @file p1000cothr.cpp
 * @brief This file contains the first of three implementations of the function start_op1_impl.
 * This implementation calls RemoteObject1::startthr_op1.
 *
 * @author Johan Vanslembrouck
 */
 
#include "p1000co.h"

/**
 * @brief
 * @param idx
 * @param in1
 * @param in2
 */
void RemoteObject1Co::start_op1_impl(const int idx, int in1, int in2)
{
    print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    m_remoteObject.startthr_op1(in1, in2,
        [this, idx](int out1, int out2, int ret1)
        {
            print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d) - handler\n", this, idx);
#if 1
            op1_ret_t op1_ret = { out1, out2, ret1 };
            this->completionHandler<op1_ret_t>(idx, op1_ret);
#else
            // Expanded implementation of completionHandler.
            // Not included here, see p1000co.cpp.
#endif
        });
}
