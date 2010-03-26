#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.61])
AC_INIT([SPINFramework], [0.2])
AC_MSG_NOTICE([Configuring the SPIN Framework])
AM_INIT_AUTOMAKE([subdir-objects])
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


LDFLAGS="$LDFLAGS -L/usr/local/lib"

AC_CONFIG_FILES([Makefile src/genwrapper/Makefile src/tinyxml/Makefile src/osgWrappers/Makefile src/spin/Makefile])

# Checks for programs.
AC_PROG_CXX
#AC_PROG_CC
AC_PROG_MAKE_SET # for recursive makefiles in subdirs
AC_PROG_INSTALL
AC_PROG_RANLIB # for making libraries
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
#PKG_CHECK_MODULES(XML, libxml-2.0 >= 2.0)
PKG_CHECK_MODULES([LIBXML2], libxml-2.0 >= 2.0)
#AC_SUBST(LIBXML_LIBS)
#AC_SUBST(LIBXML_CFLAGS)

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h stddef.h stdlib.h string.h sys/time.h])

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([gettimeofday memmove mkdir setenv sqrt strchr strpbrk uname])


# Check for Python:
AX_PYTHON

# Using AX_PYTHON_DEVEL:
#AX_PYTHON_DEVEL

# Using AX_PYTHON_EMBED:
#AX_PYTHON_DEFAULT( )
AX_PYTHON_WITH( )
AX_PYTHON_INSIST( )
AX_PYTHON_VERSION_ENSURE( [2.5] )
AX_PYTHON_CSPEC
AX_PYTHON_LSPEC

AC_MSG_NOTICE([python: ${PYTHON}])
AC_MSG_NOTICE([python bin: ${PYTHON_BIN}])
AC_MSG_NOTICE([python include: ${PYTHON_INCLUDE_DIR}])
AC_MSG_NOTICE([python CSPEC: ${PYTHON_CSPEC}])
AC_MSG_NOTICE([python LSPEC: ${PYTHON_LSPEC}])

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

#AX_BOOST_PYTHON
AX_BOOST_PYTHON_EMBED
if test "x${ac_cv_boost_python}" = "xno" ; then
    AC_MSG_ERROR([missing package: libboost-python-dev])
fi

#AC_LANG_PUSH([C++])
#AC_CHECK_HEADER([boost/python.hpp], , AC_MSG_ERROR([missing Boost Python headers]), )
#AC_LANG_POP([C++])

# Check for OpenSceneGraph:
PKG_CHECK_MODULES(OpenSceneGraph, openscenegraph >= 2.8.0, , AC_MSG_ERROR([missing OpenSceneGraph]))

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
AS_IF([test "x$with_scenic" = "xyes"],
	AC_CHECK_LIB([shared_video], [shared_video_is_present], SHARED_VIDEO_LIB="-lshared_video"; AC_DEFINE([WITH_SHARED_VIDEO],[],[A flag that compiles additional features for use with the shared_video library from Scenic]), AC_MSG_ERROR([missing Scenic shared_video library]),[-pthread])
	[with_scenic=no])

#AC_MSG_CHECKING([for Scenic shared_video support])
#AC_MSG_RESULT($with_scenic)





#AC_OUTPUT(Makefile src/genwrapper/Makefile src/osgWrappers/Makefile src/spin/Makefile)
AC_OUTPUT


