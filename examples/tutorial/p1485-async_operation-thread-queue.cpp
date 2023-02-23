/**
 * @file p1485-async_operation-thread-queue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "p1480.h"

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    print(PRI1, "main(): creating vector of random ints\n");
    //std::vector<int> values(100'000'000);
    std::vector<int> values(10'000'000);
    std::ranges::generate(values, [&]() {return ints(engine); });

    EventQueueThrFunctionVoidVoid eventQueueThr;
    Sorter sorter(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);

    print(PRI1, "main(): starting sortCoroutine\n");
    async_task<void> result = sortCoroutine(sorter, values);

    print(PRI1, "main(): running event queue\n");
    runEventQueue(eventQueueThr, 2);

    print(PRI1, "main(): resumed. Waiting for sortCoroutine to complete\n");
    result.wait();

    print(PRI1, "main(): confirming that vector is sorted\n");
    bool sorted{ std::ranges::is_sorted(values) };
    print(PRI1, "main(): values is %s sorted\n", sorted ? "" : " not");

    return 0;
}