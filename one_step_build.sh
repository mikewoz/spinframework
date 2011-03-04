#!/bin/bash

cd "`dirname $BASH_SOURCE`"
./autogen.sh
./configure
make

# alias make="make -j2" in your ~/.bashrc for more speed
