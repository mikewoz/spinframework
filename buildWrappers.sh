#!/bin/sh

GENWRAPPER_PATH=`dirname $0`/src/genwrapper

make -C ${GENWRAPPER_PATH}
doxygen ./doxygen_config

rm include/*.h~
${GENWRAPPER_PATH}/genwrapper -d . doxygen | doxygen -
${GENWRAPPER_PATH}/genwrapper -v QUIET -c genwrapper.conf doxygen .

