#!/bin/sh
# Assumption: cs1-server (or cs1l-server) and cs1-clientserver (or cs1l-clientserver) are already running

set -x

./cs1-client1
./cs1-client1a
./cs1-client1b
./cs1-client1c

./cs1-client3
./cs1-client3WA
./cs1-client3WAny

./cs1-client4obs
./cs1-client4obs2
./cs1-client4obs3

./cs1-client5thr

./cs1l-client1
./cs1l-client1a

./cs1l-client3WA
./cs1l-client3WAny
