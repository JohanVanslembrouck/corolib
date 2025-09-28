# Communicating Finite State Machines (CFSMs)

## Introduction

This directory contains examples of applications organized as finite state machines (FSMs) that communicate with each other
by posting messages to their associated message queue. Each FSM is associated with a message queue.

## p10XX

This first set of CFSMs use message ids (instead of "full" messages) to communicate.

The following files are used:

* p1000-cfsm2.h contains a simple CFSM that just responds to the messages sent by any of the CFSM1 variants.
This CFSM will be used by all 4 applications described below.
* p1000-cfsm1.h is a first implementation of CFSM1 that uses a state machine implementation
with a double select on the state and the message id.
* p1000-cfsms.cpp is an application using the CFSMs defined in p1000-cfsm1.h and p1000-cfsm2.h.

* p1010-cfsm1-co.h is an implementation of CFSM1 using coroutines and async_task.
* p1010-cfsms-co.cpp is an application using the CFSMs defined in p1010-cfsm1-co.h and p1000-cfsm2.h.

* p1012-cfsm1-co.h is an implementation of CFSM1 using coroutines and oneway_task.
* p1012-cfsms-co.cpp is an application using the CFSMs defined in p1012-cfsm1-co.h and p1000-cfsm2.h.
 
* p1014-cfsm1-co.h is an implementation of CFSM1 using coroutines and async_task with a taskholder.
* p1014-cfsms-co.cpp is an application using the CFSMs defined in p1014-cfsm1-co.h and p1000-cfsm2.h.

