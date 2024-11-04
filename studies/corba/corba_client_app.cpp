/**
 * @file corba_client_app.cpp
 * @brief
 * The file contains the source code for all invocation variants of operation1 defined in example.idl:
 * - synchronous: functions synchronous1 and synchronousN
 * - asynchronous using callback: asynchronous_callback1 and asynchronous_callbackN
 * - asynchronous using polling: asynchronous_polling1, asynchronous_pollingN and asynchronous_pollingNrev
 * - coroutine: coroutine1, coroutineN, coroutineNrev, coroutineN_when_all and coroutineN_when_any
 * 
 * A 1 at the end of a function/coroutine name indicates 1 operation invocation,
 * an N at the end of a function/coroutine name indicates multiple operation invocations,
 * rev indicates that the results will be retrieved in the reverse order as the one in the invocation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include <corolib/async_operation.h>
#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/when_any.h>
#include <corolib/print.h>

#include "interfacea_coroutine.h"
#include "interfacea.h"

using namespace moduleA;

interfaceACo interfaceACoObj;
interfaceA interfaceAObj;

const int NR = 5;

using namespace corolib;

/**
 * @brief synchronous1 calls operation1 once.
 *
 */
void synchronous1()
{
    printf("\nsynchronous1\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1;
    
    ret1 = interfaceAObj.operation1(in1, inout1, out1);
    printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
}

/**
 * @brief synchronousN calls operation1 NR times.
 * Because of the synchronous nature, NR invocations will take NR x the time
 * of a single invocation.
 *
 */
void synchronousN()
{
    printf("\nsynchronousN\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1;
    
    for (int i = 0; i < NR; ++i) {
        CORBA::Double inout1a = inout1 + i;
        ret1 = interfaceAObj.operation1(in1 + i, inout1a, out1);
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1a, out1);
    }
}

/**
 * @brief asynchronous_callback1 calls sendc_operation1 once,
 * passing it the address of an interfaceAHandler_impl object,
 * runs the event handler to await the completion of the asynchronous operation
 * and retrieves the result of the operation.
 *
 */
void asynchronous_callback1()
{
    printf("\nasynchronous_callback1\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1 = 0;
   
    interfaceAHandler_impl_ptr handler = new interfaceAHandler_impl;
    interfaceAObj.sendc_operation1(handler, in1, inout1);

    // The event queue should only contain one element
    // that corresponds to the sendc_operation1 operation.
    // No other events can be handled in this loop,
    // i.e. the application is not responsive to e.g. new inputs.
    eventqueue.run();

    if (handler->finished()) {
        handler->operation1(ret1, inout1, out1);
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
    }
    else {
        printf("operation with handler %p has not finished\n", handler);
    }

    delete handler;
}

/**
 * @brief asynchronous_callbackN calls sendc_operation1 NR times,
 * passing each call the address of an interfaceAHandler_impl object,
 * runs the event handler to await the completion of the asynchronous operations
 * and retrieves the result of the operation.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 * 
 */
void asynchronous_callbackN()
{
    printf("\nasynchronous_callbackN\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1 = 0;

    interfaceAHandler_impl_ptr handler[NR];

    for (int i = 0; i < NR; ++i) {
        CORBA::Double inout1a = inout1 + i;
        handler[i] = new interfaceAHandler_impl;
        interfaceAObj.sendc_operation1(handler[i], in1 + i, inout1a);
    }

    // The event queue should only contain NR elements
    // that correspond to the NR sendc_operation1 operations.
    // No other events can be handled in this loop,
    // i.e. the application is not responsive to e.g. new inputs.
    eventqueue.run();

    for (int i = 0; i < NR; ++i) {
        if (handler[i]->finished()) {
            handler[i]->operation1(ret1, inout1, out1);
            printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
        }
        else {
            printf("operation with handler %p has not finished\n", handler[i]);
        }
        delete handler[i];
    }
}

/**
 * @brief asynchronous_polling1 calls sendp_operation1 once.
 * Each sendp_operation1 call returns a PollerID, which is passed
 * to operation1Poller to retrieve the result of the operation.
 *
 */
void asynchronous_polling1()
{
    printf("\nasynchronous_polling1\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1 = 0;
    
    PollerID pollerId;
    CORBA::Boolean completed = false;
    CORBA::Boolean blocking = false;
    pollerId = interfaceAObj.sendp_operation1(in1, inout1);
    while (!completed)
        completed = interfaceAObj.operation1Poller(pollerId, blocking, ret1, inout1, out1);
    printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
}

/**
 * @brief asynchronous_pollingN calls sendp_operation1 NR times.
 * Each sendp_operation1 returns a PollerID, which is passed
 * to operation1Poller to retrieve the result of the operation.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 * 
 */
void asynchronous_pollingN()
{
    printf("\nasynchronous_pollingN\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1 = 0;
    
    PollerID pollerId[NR];
    
    CORBA::Boolean blocking = false;
    for (int i = 0; i < NR; ++i) {
        CORBA::Double inout1a = inout1 + i;
        pollerId[i] = interfaceAObj.sendp_operation1(in1 + i, inout1a);
    }

    for (int i = 0; i < NR; ++i) {
        CORBA::Boolean completed = false;
        while (!completed)
            completed = interfaceAObj.operation1Poller(pollerId[i], blocking, ret1, inout1, out1);
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
    }
}

/**
 * @brief asynchronous_pollingNrev calls sendp_operation1 NR times.
 * Each sendp_operation1 returns a PollerID, which is passed
 * to operation1Poller to retrieve the result of the operation.
 * The results are retrieved in the opposite order as the invocations.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 * 
 */
void asynchronous_pollingNrev()
{
    printf("\nasynchronous_pollingNrev\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Short out1 = 0;
    CORBA::Short ret1 = 0;

    PollerID pollerId[NR];

    CORBA::Boolean blocking = false;
    for (int i = 0; i < NR; ++i) {
        CORBA::Double inout1a = inout1 + i;
        pollerId[i] = interfaceAObj.sendp_operation1(in1 + i, inout1a);
    }

    for (int i = NR-1; i >= 0; --i) {
        CORBA::Boolean completed = false;
        while (!completed)
            completed = interfaceAObj.operation1Poller(pollerId[i], blocking, ret1, inout1, out1);
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", ret1, inout1, out1);
    }
}

/**
 * @brief coroutine1 calls start_operation1 once.
 * It then co_awaits the completion of this invocation.
 * The event loop is run in main(), where it can handle all events,
 * such as those corresponding to new inputs.
 * 
 */
async_task<void> coroutine1() 
{
    printf("\ncoroutine1\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;

    async_operation<operation1_result> a = interfaceACoObj.start_operation1(in1, inout1);
    operation1_result result = co_await a;
    printf("ret1 = %d, inout1 = %f, out1 = %d\n", result.m_ret_val, result.m_inout_val, result.m_out_val);

    co_return;
}

/**
 * @brief coroutineN calls start_operation1 NR times.
 * It then co_awaits the completion of these invocations.
 * The event loop is run in main(), where it can handle all events,
 * such as those corresponding to new inputs.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 *
 */
async_task<void> coroutineN()
{
    printf("\ncoroutineN\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Double inout1a;

    async_operation<operation1_result> async_ops[NR];
    
    for (int i = 0; i < NR; ++i) {
        inout1a = inout1 + i;
        async_ops[i] = interfaceACoObj.start_operation1(in1 + i, inout1a);
    }
    
    for (int i = 0; i < NR; ++i) {
        operation1_result result = co_await async_ops[i];
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", result.m_ret_val, result.m_inout_val, result.m_out_val);
    }

    co_return;
}

/**
 * @brief coroutineNrev calls start_operation1 NR times.
 * It then co_awaits the completion of these invocations in the reverse order.
 * The event loop is run in main(), where it can handle all events,
 * such as those corresponding to new inputs.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 *
 */
async_task<void> coroutineNrev()
{
    printf("\ncoroutineNrev\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Double inout1a = inout1;

    async_operation<operation1_result> async_ops[NR];

    for (int i = 0; i < NR; ++i) {
        inout1a = inout1 + i;
        async_ops[i] = interfaceACoObj.start_operation1(in1 + i, inout1a);
    }

    for (int i = NR - 1; i >= 0; --i) {
        operation1_result result = co_await async_ops[i];
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", result.m_ret_val, result.m_inout_val, result.m_out_val);
    }

    co_return;
}

/**
 * @brief coroutineN_when_all calls start_operation1 NR times.
 * It then co_awaits the completion of these invocations using when_all.
 * The event loop is run in main(), where it can handle all events,
 * such as those corresponding to new inputs.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 *
 */
async_task<void> coroutineN_when_all()
{
    printf("\ncoroutineN_when_all\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Double inout1a;

    async_operation<operation1_result> async_ops[NR];

    for (int i = 0; i < NR; ++i) {
        inout1a = inout1 + i;
        async_ops[i] = interfaceACoObj.start_operation1(in1 + i, inout1a);
    }

    async_base* pasyncsc[NR];
    for (int i = 0; i < NR; ++i)
        pasyncsc[i] = &async_ops[i];

    co_await when_all(pasyncsc, NR);

    for (int i = 0; i < NR; ++i) {
        operation1_result result = async_ops[i].get_result();
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", result.m_ret_val, result.m_inout_val, result.m_out_val);
    }

    co_return;
}

/**
 * @brief coroutineN_when_any calls start_operation1 NR times.
 * It then co_awaits the completion of these invocations using when_any.
 * The event loop is run in main(), where it can handle all events,
 * such as those corresponding to new inputs.
 * If the server(s) can process the requests in parallel,
 * the time taken for NR invocations should only be slightly
 * higher than that of a single invocation.
 *
 */
async_task<void> coroutineN_when_any()
{
    printf("\ncoroutineN_when_any\n");

    CORBA::Long in1 = 1;
    CORBA::Double inout1 = 1;
    CORBA::Double inout1a;

    async_operation<operation1_result> async_ops[NR];

    for (int i = 0; i < NR; ++i) {
        inout1a = inout1 + i;
        async_ops[i] = interfaceACoObj.start_operation1(in1 + i, inout1a);
    }

    async_base* pasyncsc[NR];
    for (int i = 0; i < NR; ++i)
        pasyncsc[i] = &async_ops[i];
    when_any wa(pasyncsc, NR);

    for (int i = 0; i < NR; ++i) {
        int r = co_await wa;
        operation1_result result = async_ops[r].get_result();
        printf("ret1 = %d, inout1 = %f, out1 = %d\n", result.m_ret_val, result.m_inout_val, result.m_out_val);
    }

    co_return;
}

int main()
{
    synchronous1();
    synchronousN();

    asynchronous_callback1();
    asynchronous_callbackN();

    asynchronous_polling1();
    asynchronous_pollingN();
    asynchronous_pollingNrev();

    async_task<void> t1 = coroutine1();
    eventqueue.run();
    t1.wait();

    async_task<void> t2 = coroutineN();
    eventqueue.run();
    t2.wait();

    async_task<void> t3 = coroutineNrev();
    eventqueue.run();
    t3.wait();

    async_task<void> t4 = coroutineN_when_all();
    eventqueue.run();
    t4.wait();

    async_task<void> t5 = coroutineN_when_any();
    eventqueue.run();
    t5.wait();

    printf("\n");
    return 0;
}