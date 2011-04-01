#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.61])
AC_INIT(SPINFRAMEWORK, [0.3.3], [https://sourceforge.net/apps/trac/spinframework/], [spinframework], [http://www.spinframework.org])
AC_MSG_NOTICE([Configuring the SPIN Framework])
AM_INIT_AUTOMAKE([subdir-objects -Wno-portability])
#AC_CONFIG_SRCDIR([include/ModelNode.h])
AC_CONFIG_HEADERS([include/config.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST # fills $host variable

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_CHECK_PROGS([DOXYGEN], [doxygen])

if test -z "$DOXYGEN";
    then AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test -n "$DOXYGEN"])

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

# Special cases
case "$host" in *-darwin* | *-macos10*)
  CPPFLAGS="$CPPFLAGS -D__Darwin"
  LDFLAGS="-undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CPPFLAGS="$CPPFLAGS -I/usr/local/include -I/opt/local/include"
    LDFLAGS="$LDFLAGS -L/usr/local/lib -L/opt/local/lib"
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
# LDFLAGS="$LDFLAGS -L/usr/local/lib -L/usr/local/lib64"

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

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h stddef.h stdlib.h string.h sys/time.h])

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([gettimeofday memmove mkdir setenv sqrt strchr strpbrk uname])

# Check for Python:
AM_PATH_PYTHON

PYTHON_CPPFLAGS="`python-config --cflags`"
PYTHON_CPPFLAGS=${PYTHON_CPPFLAGS//"-Wstrict-prototypes"/}
PYTHON_LDFLAGS="`python-config --ldflags`"
AC_SUBST(PYTHON_CPPFLAGS)
AC_SUBST(PYTHON_LDFLAGS)

# manually set pythondir (where site-packages is located)
# and pyexecdir for OSX
case "$host" in *-darwin* | *-macos10*)
  pythondir=`python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
  pyexecdir=`python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
esac

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

PKG_CHECK_MODULES(OSG, openscenegraph >= 2.8.0, , AC_MSG_ERROR([missing OpenSceneGraph]))

PKG_CHECK_MODULES(CPPINTROSPECTION, cppintrospection-2.11 >= 2.11.0, , AC_MSG_ERROR([missing cppintrospection]))

PKG_CHECK_MODULES(OPENTHREADS, openthreads >= 2.4.0, , AC_MSG_ERROR([missing OpenThreads]))

PKG_CHECK_MODULES(LIBLO, liblo, , AC_MSG_ERROR([missing liblo]))

# FIXME: scenic should use pkg-config to say where shared video is
OLD_LDFLAGS=$LDFLAGS
LDFLAGS="$LDFLAGS -L/usr/local/lib/scenic -L/usr/lib/scenic"
AC_CHECK_LIB([shared_video], [shared_video_is_present], 
        SHARED_VIDEO_LIB="-lshared_video"; 
        [AC_DEFINE([WITH_SHARED_VIDEO],[],
            [A flag that compiles additional features for use with the shared_video library from Scenic]) AC_MSG_NOTICE([Using scenic shared video library])], 
        [AC_MSG_WARN([missing Scenic shared_video library]); 
        LDFLAGS=$OLD_LDFLAGS],[-pthread])

AC_SUBST([SHARED_VIDEO_LIB])

AC_ARG_ENABLE([editor],
    AS_HELP_STRING([--enable-editor], [Enables compilation of the graphical editor]),
    [with_editor=$enableval],
    [with_editor=no])
AM_CONDITIONAL(ENABLE_EDITOR, test "x$with_editor" = xyes)

if test "x$with_editor" = xyes; then
    AC_MSG_NOTICE([Enabling SPIN Editor])
    AM_OPTIONS_WXCONFIG
    reqwx=2.4.0
    AM_PATH_WXCONFIG($reqwx, wxWin=1)
    if test "$wxWin" != 1; then
        AC_MSG_ERROR([wxWidgets must be installed on your system. Please check that wx-config is in path, the directory where wxWidgets libraries are installed (returned by 'wx-config --libs' or 'wx-config --static --libs' command) is in LD_LIBRARY_PATH or equivalent variable and wxWidgets version is $reqwx or above.])
    fi
fi

AC_MSG_NOTICE([============================================])
#AS_IF([test "x$with_scenic" = "xyes"], AC_MSG_NOTICE([Using scenic]), AC_MSG_NOTICE([Not using scenic]))
AC_MSG_NOTICE([PYTHON_LIB:          ${PYTHON_LIB}])
AC_MSG_NOTICE([PYTHON_INCLUDE_DIR:  ${PYTHON_INCLUDE_DIR}])
AC_MSG_NOTICE([PYTHON_CPPFLAGS:     ${PYTHON_CPPFLAGS}])
AC_MSG_NOTICE([PYTHON_LDFLAGS:      ${PYTHON_LDFLAGS}])
AC_MSG_NOTICE([BOOST_LDFLAGS:       ${BOOST_LDFLAGS}])
AC_MSG_NOTICE([BOOST_CPPFLAGS:      ${BOOST_CPPFLAGS}])
AC_MSG_NOTICE([BOOST_PYTHON_LIB:    ${BOOST_PYTHON_LIB}])
AC_MSG_NOTICE([WX_CPPFLAGS:         ${WX_CPPFLAGS}])
AC_MSG_NOTICE([WX_LIBS:             ${WX_LIBS}])
AC_MSG_NOTICE([pythondir:           ${pythondir}])
AC_MSG_NOTICE([pythonexecdir:       ${pyexecdir}])
AC_MSG_NOTICE([============================================])

AC_CONFIG_FILES([
        Makefile 
        doxyfile
        src/Makefile 
        src/tinyxml/Makefile 
        src/osgWrappers/introspection/Makefile 
        src/spin/Makefile
        src/spineditor/Makefile
        src/spineditor/wxsmith/Makefile
        src/Resources/Makefile
        src/Resources/images/Makefile
        src/Resources/desktopentries/Makefile
        src/Resources/fonts/Makefile
        src/Resources/scripts/Makefile
        tests/Makefile
        ])

AC_OUTPUT
