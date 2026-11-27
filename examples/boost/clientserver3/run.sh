#!/bin/bash

runserver() {
    rm -f $1.txt
    $1 > $1.txt 2>&1 &
    SERVER_PID=$!
    echo $SERVER_PID
    ./cs3-client2
    echo $SERVER_PID
    kill -9 $SERVER_PID
    sleep 2
    cat $1.txt
    sleep 1
}

runserver "./cs3-server"
runserver "./cs3-server2"
runserver "./cs3-server3"
runserver "./cs3-server4"
runserver "./cs3-server5"
runserver "./cs3-server6"
runserver "./cs3-server7"
runserver "./cs3-server8"
runserver "./cs3-server8a"
runserver "./cs3-server9"
runserver "./cs3-server9a"
runserver "./cs3-server10"
runserver "./cs3-server10a"
runserver "./cs3-server11a"
