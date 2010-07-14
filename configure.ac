#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.61])
AC_INIT(SPINFramework, [0.3.2], [https://sourceforge.net/apps/trac/spinframework/])
AC_MSG_NOTICE([Configuring the SPIN Framework])
AM_INIT_AUTOMAKE([subdir-objects -Wno-portability])
#AC_CONFIG_SRCDIR([include/ModelNode.h])
AC_CONFIG_HEADERS([include/config.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST # fills $host variable

# Special cases
case "$host" in *-darwin* | *-macos10*)
  if test -d /opt/local ; then
    CPPFLAGS="$CPPFLAGS -I/opt/local/include"
    LDFLAGS="$LDFLAGS -L/opt/local/lib"
  elif test -d /sw ; then
    CPPFLAGS="$CPPFLAGS -I/sw/include"
    LDFLAGS="$LDFLAGS -L/sw/lib"
  fi
  ;;
esac

# OSX does not include /usr/local/lib/pkgconfig path, so we export it here
# (for everyone, just in case)
PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export PKG_CONFIG_PATH

CXXFLAGS+=" -Wall -Wextra -Wfatal-errors"
LDFLAGS="$LDFLAGS -L/usr/local/lib -L/usr/local/lib64"

AC_CONFIG_FILES([
        Makefile 
        src/Makefile 
        src/tinyxml/Makefile 
        src/osgWrappers/introspection/Makefile 
        src/spin/Makefile])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES([ptrdiff_t])

# OpenGL:
AX_CHECK_GLUT

# Checks for libraries
AC_CHECK_LIB([m],[sqrt])
PKG_CHECK_MODULES(XML, libxml-2.0 >= 2.0)
PKG_CHECK_MODULES([LIBXML2], libxml-2.0 >= 2.0)
#AC_SUBST(LIBXML_LIBS)
#AC_SUBST(LIBXML_CFLAGS)

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h stddef.h stdlib.h string.h sys/time.h])

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([gettimeofday memmove mkdir setenv sqrt strchr strpbrk uname])


# Check for Python:
#AX_PYTHON
AM_PATH_PYTHON

PYTHON_CPPFLAGS="`python-config --cflags`"
PYTHON_CPPFLAGS=${PYTHON_CPPFLAGS//"-Wstrict-prototypes"/}
PYTHON_LDFLAGS="`python-config --ldflags`"
AC_SUBST(PYTHON_CPPFLAGS)
AC_SUBST(PYTHON_LDFLAGS)

#AC_MSG_RESULT(["    PYTHON_LDFLAGS: $PYTHON_LDFLAGS"])
#AC_MSG_RESULT(["    PYTHON_CPPFLAGS: $PYTHON_CPPFLAGS"])


# manually set pythondir (where site-packages is located)
# and pyexecdir for OSX
case "$host" in *-darwin* | *-macos10*)
  pythondir=`python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
  pyexecdir=`python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
esac

# Using AX_PYTHON_DEVEL:
#AX_PYTHON_DEVEL

# Using AX_PYTHON_EMBED:
#AX_PYTHON_WITH
#AX_PYTHON_PATH
#AX_PYTHON_VERSION_ENSURE( [2.5] )
#AX_PYTHON_CSPEC
#AX_PYTHON_LSPEC

# Another python check: 
#AM_PATH_PYTHON
#AM_CHECK_PYTHON_HEADERS
#AM_CHECK_PYTHON_LIB
#if test -z "$PYTHON_INCLUDES" || test -z "$PYTHON_LIBS"; then
#AC_MSG_ERROR([Could not find Python])
#fi

# Check for boost:
AX_BOOST_BASE([1.35])


AX_BOOST_REGEX
if test "x${ax_cv_boost_regex}" = "xno" ; then
    AC_MSG_ERROR([missing package: libboost-regex-dev])
fi

AX_BOOST_FILESYSTEM
if test "x${ax_cv_boost_filesystem}" = "xno" ; then
        AC_MSG_ERROR([missing package: libboost-filesystem-dev])
fi

AX_BOOST_THREAD
if test "x${ax_cv_boost_thread}" = "xno" ; then
    AC_MSG_ERROR([missing package: libboost-thread-dev])
fi

#AC_CHECK_LIB(boost_python-mt, main, AC_MSG_NOTICE([boost_python? yep]), AC_MSG_NOTICE([boost_python? nada]), -l$PYTHON_LIB)

AX_BOOST_PYTHON
if test -z "$BOOST_PYTHON_LIB" ; then
    AC_MSG_ERROR([missing package. libboost-python-dev.])
fi

# Check for OpenSceneGraph:
#PKG_CHECK_MODULES(OpenSceneGraph, openscenegraph >= 2.8.0, , AC_MSG_ERROR([missing OpenSceneGraph]))
AC_CHECK_LIB(osg, osgGetVersion, , AC_MSG_ERROR([missing OpenSceneGraph]),)

# and osgIntrospection:
AC_CHECK_LIB(osgIntrospection, osgIntrospectionGetVersion, OSGINTROSPECTION_LIB="-losgIntrospection", AC_MSG_ERROR([missing OpenSceneGraph Introspection library]),)
AC_SUBST([OSGINTROSPECTION_LIB])

# another way to check for OSG libs?:
#AC_CHECK_LIB(osg,osgGetVersion)
#AC_CHECK_LIB(OpenThreads,OpenThreadsGetVersion)


# Mac OS X specific configure options
#AC_ARG_WITH(osg_framework, [ --with-osg-framework=PREFIX Specify the prefix path to osg frameworks [default=standard framework paths]])
#AC_CHECK_FRAMEWORK(osg, [#include <osg/Version>])
#AC_CHECK_FRAMEWORK(OpenThreads, [#include <OpenThreads/Version>])


# check for liblo:
AC_CHECK_LIB(lo, lo_send_internal, LIBLO_LDFLAGS="-llo", AC_MSG_ERROR([missing liblo OSC library]),)
AC_SUBST([LIBLO_LDFLAGS])

# check for scenic shared video stuff:
AC_ARG_ENABLE([scenic],
	AS_HELP_STRING([--enable-scenic], [Enables SharedVideoTexture by linking with the shared_video library from Scenic]),
	[with_scenic=$enableval],
	[with_scenic=no])

# FIXME: scenic should use pkg-config to say where shared video is
AS_IF([test "x$with_scenic" = "xyes"],[
    LDFLAGS="$LDFLAGS -L/usr/local/lib/scenic -L/usr/lib/scenic" 
	AC_CHECK_LIB([shared_video], [shared_video_is_present], SHARED_VIDEO_LIB="-lshared_video"; 
        AC_DEFINE([WITH_SHARED_VIDEO],[],[A flag that compiles additional features for use with the shared_video library from Scenic]), 
        AC_MSG_ERROR([missing Scenic shared_video library]),[-pthread])], [with_scenic=no])

AC_SUBST([SHARED_VIDEO_LIB])

AC_ARG_ENABLE([fmod],
	AS_HELP_STRING([--enable-fmod], [Enables audio rendering via FMOD]),
	[with_fmod=$enableval],
	[with_fmod=no])

#AC_PUSH_LANG(C++)
#AS_IF([test "x$with_fmod" = "xyes"],
#	AC_CHECK_LIB([fmodex], [], FMOD_LDFLAGS="-lfmodex"; CPPFLAGS="$CPPFLAGS -I/Developer/FMOD\ Programmers\ API\ Mac/api/inc/"; AC_DEFINE([USE_FMOD],[],[A flag that enables audio rendering features using the FMOD engine]), AC_MSG_ERROR([missing FMOD libraries]),)
#	[with_fmod=no])
#AC_POP_LANG(C++)

AC_ARG_ENABLE([editor],
	AS_HELP_STRING([--enable-editor], [Enables compilation of the graphical editor]),
	[with_gui=$enableval],
	[with_gui=no])

# GNU help2man creates man pages from --help output; in many cases, this
# is sufficient, and obviates the need to maintain man pages separately.
# However, this means invoking executables, which we generally cannot do
# when cross-compiling, so we test to avoid that (the variable
# "cross_compiling" is set by AC_PROG_CC).
if test $cross_compiling = no; then
    AM_MISSING_PROG(HELP2MAN, help2man)
else
    HELP2MAN=:
fi

AM_CONDITIONAL(ENABLE_EDITOR, [test "x$with_gui" = xyes])


#AC_MSG_CHECKING([for Scenic shared_video support])
#AC_MSG_RESULT($with_scenic)

AC_MSG_NOTICE([============================================])
AS_IF([test "x$with_scenic" = "xyes"], AC_MSG_NOTICE([Using scenic]), AC_MSG_NOTICE([Not using scenic]))
AC_MSG_NOTICE([PYTHON_LIB:          ${PYTHON_LIB}])
AC_MSG_NOTICE([PYTHON_INCLUDE_DIR:  ${PYTHON_INCLUDE_DIR}])
AC_MSG_NOTICE([PYTHON_CPPFLAGS:     ${PYTHON_CPPFLAGS}])
AC_MSG_NOTICE([PYTHON_LDFLAGS:      ${PYTHON_LDFLAGS}])
AC_MSG_NOTICE([BOOST_LDFLAGS:       ${BOOST_LDFLAGS}])
AC_MSG_NOTICE([BOOST_CPPFLAGS:      ${BOOST_CPPFLAGS}])
AC_MSG_NOTICE([BOOST_PYTHON_LIB:    ${BOOST_PYTHON_LIB}])
AC_MSG_NOTICE([pythondir:           ${pythondir}])
AC_MSG_NOTICE([pythonexecdir:       ${pyexecdir}])
AC_MSG_NOTICE([============================================])

#AC_OUTPUT(Makefile src/genwrapper/Makefile src/osgWrappers/Makefile src/spin/Makefile)
AC_OUTPUT
