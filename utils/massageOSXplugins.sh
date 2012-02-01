#!/bin/sh
#
# This scripts changes the library names in all osgPlugins so that they can 
# work BOTH in a command-line AND XCode .app bundle. The script runs
# install_name_tool on all plugins and removes all paths from the library
# names. As a result, you need to use osgDB::Registry to manually add paths
# within the app. For example:
#
#   osgDB::FilePathList osgLibPaths;
#   osgLibPaths.push_back([[[NSBundle mainBundle] bundlePath] UTF8String]);
#   osgLibPaths.push_back([[[NSBundle mainBundle] bundlePath] UTF8String]+"/Contents/PlugIns");
#   osgDB::Registry::instance()->setLibraryFilePathList(osgLibPaths);
#
# NOTE: you must run this script with sudo
#
# Author: Mike Wozniewski <mike@mikewoz.com>


# Set the versions here:
OSG_VERSION="2.9.8"
OSG_LIBVERSION="66"
OPENTHREADS_VERSION="2.5.0"
OPENTHREADS_LIBVERSION="12"

# --------------------------------------------------------

OSG_LIBS=`ls /usr/local/lib/libosg*.${OSG_VERSION}.dylib`
#OSG_PLUGINS=`ls /usr/local/lib/osgPlugins-${OSG_VERSION}/*.so`
OSG_PLUGINS=`ls /Users/mikewoz/Library/Developer/Xcode/DerivedData/spinframework-eftvacjfdotaeufndczqnkbnyazw/Build/Products/Debug/spinviewerUI.app/Contents/Plugins/*.so`

#echo "OSG_LIBS: ${OSG_LIBS}"
echo "OSG_PLUGINS: ${OSG_PLUGINS}"

REGEX="libOpenThreads.*|libosg.*dylib"

for i in $OSG_PLUGINS
do
  for x in `otool -L ${i}`
  do
    if [[ $x =~ $REGEX ]]
    then
    #basename=$(basename $x)
    #lib=${basename%%.*}
    #osglib=${lib#lib}
    #echo "plugin:${i} osglib:${osglib}"

      cmd="install_name_tool -change ${x} @rpath/${x} ${i}"
    echo $cmd
    `$cmd`
    fi
  done
done

