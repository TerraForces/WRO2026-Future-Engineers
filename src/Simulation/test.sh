#! /bin/bash
rm ./sim 2> /dev/null
g++ sim.cpp ../Teensy/src/main.cpp -o sim -Ofast -I.
./sim $0 $@