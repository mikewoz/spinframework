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

command -v genwrapper >/dev/null 2>&1 || { echo "genwrapper is not installed; please install cppintrospection before proceeding." >&2; exit 1; }

#make -C ${GENWRAPPER_PATH}
doxygen ./doxygen_config

# check if there are any temp headers and remove them
# (the check is silent, so if none exist, there will not be a warning)
if ls include/*.h~ > /dev/null 2>&1; then
  rm include/*.h~
fi

#genwrapper -d . doxygen | doxygen -
genwrapper -v QUIET -c genwrapper.conf doxygen .
#genwrapper -v DEBUG -c genwrapper.conf doxygen .
rm -rf src/osgWrappers/introspection/home
