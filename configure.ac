#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.61])

# spin package version number, (as distinct from shared library version)
# An odd micro number indicates in-progress development from Git
# An even micro number indicates a released version
#
# Making a point release:
# - increase spin_micro_version to the next even number
# - increase spin_interface_age to the next even number
#   UNLESS there was an API addition/deprecation, in which case
#   - set spin_interface_age to 0
# After the release:
# - increase spin_micro_version to the next odd number
# - increase spin_interface_version to the next odd number
m4_define([spin_major_version], [0])
m4_define([spin_minor_version], [3])
m4_define([spin_micro_version], [10])

m4_define([spin_release_status],
          [m4_if(m4_eval(spin_micro_version % 2), [1], [git],
                 [m4_if(m4_eval(spin_minor_version % 2), [1], [snapshot],
                                                                 [release])])])

m4_define([spin_version], [spin_major_version.spin_minor_version.spin_micro_version])

# change this only when breaking the API
m4_define([spin_api_version], [0.3])

# increase the interface age by 1 for each release; if the API changes,
# set to 0. interface_age and binary_age are used to create the soname
# of the shared object:
#
#  (<minor> * 100 + <micro>) - <interface_age>
#
# this allows using the same soname for different micro-releases in case
# no API was added or deprecated. for instance:
#
#   spin 1.2.0  -> 100 * 2 + 0  = 200, interface age = 0 -> 200
#   spin 1.2.2  -> 100 * 2 + 2  = 202, interface age = 2 -> 200
#   spin 1.2.4  -> 100 * 2 + 4  = 204, interface age = 4 -> 200
#   [ API addition, deprecation ]
#   spin 1.2.6  -> 100 * 2 + 6  = 206, interface age = 0 -> 206
#   spin 1.2.8  -> 100 * 2 + 8  = 208, interface age = 2 -> 206
#   spin 1.2.10 -> 100 * 2 + 10 = 210, interface age = 4 -> 206
#   ...
#
m4_define([spin_interface_age], [0])
m4_define([spin_binary_age], [m4_eval(100 * spin_minor_version + spin_micro_version)])

AC_PREREQ([2.63])
AC_INIT(SPINFRAMEWORK,
        [spin_version],
        [http://www.spinframework.org],
        [spinframework],
        [http://github.com/mikewoz/spinframework/issues])

AC_MSG_NOTICE([Configuring the SPIN Framework])
AM_INIT_AUTOMAKE([subdir-objects -Wno-portability])
#AC_CONFIG_SRCDIR([include/ModelNode.h])
AC_CONFIG_HEADERS([include/config.h])
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST # fills $host variable

AC_SUBST([SPIN_MAJOR_VERSION],  [spin_major_version])
AC_SUBST([SPIN_MINOR_VERSION],  [spin_minor_version])
AC_SUBST([SPIN_MICRO_VERSION],  [spin_micro_version])
AC_SUBST([SPIN_VERSION],        [spin_version])
AC_SUBST([SPIN_API_VERSION],    [spin_api_version])
AC_SUBST([SPIN_API_VERSION_AM], [$SPIN_MAJOR_VERSION\_0])
AC_SUBST([SPIN_RELEASE_STATUS], [spin_release_status])

m4_define([lt_current],  [m4_eval(100 * spin_minor_version + spin_micro_version - spin_interface_age)])
m4_define([lt_revision], [spin_interface_age])
m4_define([lt_age],      [m4_eval(spin_binary_age - spin_interface_age)])
SPIN_LT_CURRENT=lt_current
SPIN_LT_REV=lt_revision
SPIN_LT_AGE=lt_age
SPIN_LT_VERSION="$SPIN_LT_CURRENT:$SPIN_LT_REV:$SPIN_LT_AGE"
AC_SUBST([SPIN_LT_VERSION], "$SPIN_LT_VERSION")
SPIN_LT_LDFLAGS="-version-info $SPIN_LT_VERSION"

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL

AC_PROG_LIBTOOL
# LT_INIT should be used instead of AC_PROG_LIBTOOL
#LT_PREREQ([2.4])
#LT_INIT

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
  LIBTOOL="glibtool"
  #LIBTOOL="libtool"
  CPPFLAGS="$CPPFLAGS -D__Darwin"
  LDFLAGS="-undefined dynamic_lookup -Wl,-headerpad_max_install_names"
  if test -d /opt/local ; then
    CPPFLAGS="$CPPFLAGS -I/usr/local/include -I/opt/local/include"
    LDFLAGS="$LDFLAGS -L/usr/local/lib -L/opt/local/lib"
  elif test -d /sw ; then
    CPPFLAGS="$CPPFLAGS -I/sw/include"
    LDFLAGS="$LDFLAGS -L/sw/lib"
  fi
  # OSX does not include /usr/local/lib/pkgconfig path, so we export it here
  # (for everyone, just in case)
  # TODO: also need to add the path for linux64
  PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_PATH
  ;;
esac

CXXFLAGS+=" -Wall -Wextra -Wfatal-errors"
LDFLAGS="$LDFLAGS -L/usr/local/lib -L/usr/local/lib64"

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES([ptrdiff_t])
AC_PROG_LN_S

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


BOOST_REQUIRE([1.35])
BOOST_REGEX
BOOST_SYSTEM
BOOST_FILESYSTEM
#BOOST_THREAD
BOOST_PROGRAM_OPTIONS
BOOST_PYTHON

#AC_MSG_NOTICE([BOOST_LDFLAGS:        ${BOOST_LDFLAGS}])
AC_MSG_NOTICE([BOOST_CPPFLAGS:             ${BOOST_CPPFLAGS}])
AC_MSG_NOTICE([BOOST_SYSTEM_LIBS:          ${BOOST_SYSTEM_LIBS}])
AC_MSG_NOTICE([BOOST_FILESYSTEM_LIBS:      ${BOOST_FILESYSTEM_LIBS}])
AC_MSG_NOTICE([BOOST_PROGRAM_OPTIONS_LIBS: ${BOOST_PROGRAM_OPTIONS_LIBS}])
AC_MSG_NOTICE([BOOST_PYTHON_LIBS:          ${BOOST_PYTHON_LIBS}])
AC_MSG_NOTICE([BOOST_PYTHON_LDFLAGS:       ${BOOST_PYTHON_LDFLAGS}])




PKG_CHECK_MODULES(OSG, openscenegraph >= 2.8.0, , AC_MSG_ERROR([missing OpenSceneGraph]))

PKG_CHECK_MODULES(CPPINTROSPECTION, cppintrospection-3.0, , AC_MSG_ERROR([missing cppintrospection]))

PKG_CHECK_MODULES(OPENTHREADS, openthreads >= 2.4.0, , AC_MSG_ERROR([missing OpenThreads]))

PKG_CHECK_MODULES(LIBLO, liblo, , AC_MSG_ERROR([missing liblo]))

#PKG_CHECK_MODULES(SHAREDVIDEO, shared-video-0.6, 
#        [AC_DEFINE([WITH_SHARED_VIDEO],[],
#            [A flag that compiles additional features for use with the shared_video library from Scenic]) AC_MSG_NOTICE([Using scenic shared video library])], 
#        [AC_MSG_WARN([missing Scenic shared_video library])])


PKG_CHECK_MODULES(SHAREDVIDEO, shared-video-0.6, have_sharedvideo=true, have_sharedvideo=false)
if test "x${have_sharedvideo}" = "xfalse" ; then
    AC_MSG_WARN([missing package: libshared-video-0.6-dev ])
else
    AC_DEFINE([WITH_SHARED_VIDEO],[],[A flag that compiles additional features for use with the shared_video library from Scenic]) AC_MSG_NOTICE([Using scenic shared video library])
    AC_MSG_NOTICE([SHAREDVIDEO_CFLAGS:    ${SHAREDVIDEO_CFLAGS}])
    AC_MSG_NOTICE([SHAREDVIDEO_LIBS:      ${SHAREDVIDEO_LIBS}])
fi

PKG_CHECK_MODULES(SPATOSC, spatosc-0.3, have_spatosc=true, have_spatosc=false)
if test "x${have_spatosc}" = "xfalse" ; then
    AC_MSG_WARN([missing package: libspatosc-0.3])
else
    AC_DEFINE([WITH_SPATOSC],[1],[Enables linking with the SpatOSC library so that compatible audio renderers can be used for 3D sound spatialization]) AC_MSG_NOTICE([Enabled SpatOSC for audio])
    AC_MSG_NOTICE([SPATOSC_CFLAGS:        ${SPATOSC_CFLAGS}])
    AC_MSG_NOTICE([SPATOSC_LIBS:          ${SPATOSC_LIBS}])
fi

AC_ARG_ENABLE([editor],
    AS_HELP_STRING([--enable-editor], [Enables compilation of the graphical editor]),
    [with_editor=$enableval],
    [with_editor=no])
AM_CONDITIONAL(ENABLE_EDITOR, test "x$with_editor" = xyes)

if test "x$with_editor" = xyes; then
    AC_MSG_NOTICE([Enabling SPIN Editor])
    AM_OPTIONS_WXCONFIG
    reqwx=2.9.0
    AM_PATH_WXCONFIG($reqwx, wxWin=1)
    if test "$wxWin" != 1; then
        AC_MSG_ERROR([wxWidgets must be installed on your system. See README. Please check that wx-config is in path, the directory where wxWidgets libraries are installed (returned by 'wx-config --libs' or 'wx-config --static --libs' command) is in LD_LIBRARY_PATH or equivalent variable and wxWidgets version is $reqwx or above.])
    fi
fi

AC_MSG_NOTICE([============================================])
#AS_IF([test "x$with_scenic" = "xyes"], AC_MSG_NOTICE([Using scenic]), AC_MSG_NOTICE([Not using scenic]))
AC_MSG_NOTICE([PYTHON_LIB:          ${PYTHON_LIB}])
AC_MSG_NOTICE([PYTHON_INCLUDE_DIR:  ${PYTHON_INCLUDE_DIR}])
AC_MSG_NOTICE([PYTHON_CPPFLAGS:     ${PYTHON_CPPFLAGS}])
AC_MSG_NOTICE([PYTHON_LDFLAGS:      ${PYTHON_LDFLAGS}])
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
        src/Resources/Makefile
        src/Resources/images/Makefile
        src/Resources/desktopentries/Makefile
        src/Resources/camconfig/Makefile
        src/Resources/fonts/Makefile
        src/Resources/scripts/Makefile
        src/editor/Makefile
        tests/Makefile
        spin-${SPIN_API_VERSION}.pc:spin.pc.in
        completion/Makefile
        ])

AC_OUTPUT
