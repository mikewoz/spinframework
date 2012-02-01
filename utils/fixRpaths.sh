#!/bin/sh

LIBPATH="."
LIBS="spinviewerUI.app/Contents/Frameworks/libSPIN.dylib test.dylib"

REGEX="/usr/local/lib|/opt/local/lib.*dylib"
for LIB in $LIBS
do
    echo "----------------------------------------"
    echo "-- FIXING RPATH IN $LIB:"
    for i in `otool -L ${LIBPATH}/${LIB}`
    do
        if [[ $i =~ $REGEX ]]
        then
        file=`basename ${i}`
        ext=${file##*.}
        file=${file%.*}
        cmd="install_name_tool -change ${i} @rpath/$file $LIB"
        echo "$cmd"
        `${cmd}`
    fi
    done
done
echo ""
