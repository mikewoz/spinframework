#!/bin/bash

#if [ `uname -s` = 'Darwin' ]; then
#  LIBTOOLIZE="glibtoolize"
#else
#  LIBTOOLIZE="libtoolize"
#fi


#./buildWrappers.sh

# could be replaced with autoreconf -fivI m4 (verbose, force rebuild of ltmain, .in files, etc.)
#${LIBTOOLIZE} --force
autoreconf --install

if [ $? != 0 ]; then 
    echo "autoreconf return value is $?"
    exit 1
fi

