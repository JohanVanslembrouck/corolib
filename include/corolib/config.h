/**
 * @file config.h
 * @brief This file allows configuring the compilation of corolib
 * using either the async_base base class (original configuration) or
 * using the when_type concept (preferred configuration)
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _CONFiG_H__
#define _CONFiG_H__

/*
| USE_ASYNC_BASE | USE_WHEN_TYPE | comment                                 |
| -------------- | ------------- | --------------------------------------- |
| 1              | 0             | original configuration                  |
| 1              | 1             | intermediate configuration              |
| 0              | 1             | preferred configuration                 |
| 0              | 0             | not possible: when_all and when_any     |
|                |               | use async_base in their implementation. |
*/

#define USE_ASYNC_BASE 1
#define USE_WHEN_TYPE 0

#if USE_ASYNC_BASE
#define override_if_async_base override
#else
#define override_if_async_base
#endif

#endif
