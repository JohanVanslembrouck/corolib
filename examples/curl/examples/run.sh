#!/bin/sh
set -x

./multi-single
./multi-single-class
./multi-single-coroutine

./multi-double
./multi-double-class
./multi-double-coroutine
