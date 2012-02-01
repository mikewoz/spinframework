#!/bin/sh

# define path to genwrapper source here:
#GENWRAPPER_PATH=/path/to/your/genwrapper/folder
#GENWRAPPER_PATH=`dirname $0`/src/genwrapper

OS="$(uname)"
if [ $OS = Darwin ] ; then
    #GENWRAPPER_PATH=/opt/local/bin
    GENWRAPPER_PATH=/usr/local/bin
    #GENWRAPPER_PATH=`dirname $0`/src/genwrapper
fi


PATH=${PATH}:${GENWRAPPER_PATH}
echo "PATH=${PATH}"

type -P genwrapper &>/dev/null || { echo "genwrapper is not installed; please install before proceeding." >&2; exit 1; }

#make -C ${GENWRAPPER_PATH}
doxygen ./doxygen_config

rm include/*.h~
genwrapper -d . doxygen | doxygen -
genwrapper -v QUIET -c genwrapper.conf doxygen .
#genwrapper -v DEBUG -c genwrapper.conf doxygen .
rm -rf src/osgWrappers/introspection/home
