#!/bin/sh

# define path to genwrapper source here:
#GENWRAPPER_PATH=/path/to/your/genwrapper/folder
#GENWRAPPER_PATH=`dirname $0`/src/genwrapper



PATH=${PATH}:${GENWRAPPER_PATH}

type -P genwrapper &>/dev/null || { echo "genwrapper is not installed; please install before proceeding." >&2; exit 1; }

#make -C ${GENWRAPPER_PATH}
doxygen ./doxygen_config

rm include/*.h~
genwrapper -d . doxygen | doxygen -
genwrapper -v QUIET -c genwrapper.conf doxygen .
#genwrapper -v DEBUG -c genwrapper.conf doxygen .

