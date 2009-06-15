#!/bin/sh

# test existence of genwrapper
if [ ! -x "genwrapper" ]; then
    echo "genwrapper binary not found"
    echo "have you make it ?? :-)"
    exit 1
fi


# test the definition of $OSGHOME
if [ -z "$OSGHOME" ]; then
    echo "=== >> you must define the OSGHOME environment variable << ===" 
    echo "           sh example   : set OSGHOME=/directory/to/OSG/source"
    echo "           tcsh example : setenv OSGHOME /directory/to/OSG/source"
    exit 1
else
    echo "use $OSGHOME for OpenSceneGraph source directory"
fi


# test existence of doxygen
command -v doxygen > /dev/null
if [ $? == 1 ]; then
    echo "doxygen can't be found. Check if it is in the path."
    exit 1
fi


# test the doxygen version
DOXY_VERSION=`doxygen | grep version | cut -d' ' -f3`

DOXY_VERSION_MAJ=`echo $DOXY_VERSION | cut -d'.' -f1`
DOXY_VERSION_MIN=`echo $DOXY_VERSION | cut -d'.' -f2`
DOXY_VERSION_BUI=`echo $DOXY_VERSION | cut -d'.' -f3`

if [ $DOXY_VERSION_MAJ -ne 1 ] || [ $DOXY_VERSION_MIN -ne 4 ] || [ $DOXY_VERSION_BUI -lt 1 ]  || [ $DOXY_VERSION_BUI -gt 7 ]; then
    echo "Your Doxygen version is $DOXY_VERSION."
    echo "You must use the doxygen version 1.4.x with x between [1-7]."
    echo "If you have another version of doxygen, try it and"
    echo " report on the osg-mailing list if the wrapper compilation failed or success."
    exit 1
fi



# generate the xml documentation    
./genwrapper -d $OSGHOME | doxygen -


#generate the wrapper
./genwrapper -lpm $OSGHOME
