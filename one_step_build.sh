#!/bin/bash

PKG_CONFIG_PATH=/usr/local/lib64/pkgconfig:$PKG_CONFIG_PATH
export PKG_CONFIG_PATH # ugliest hack
LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib64"
export LD_LIBRARY_PATH
cd "`dirname $BASH_SOURCE`"
./autogen.sh
./configure
make


