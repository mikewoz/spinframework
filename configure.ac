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
m4_define([spin_major_version], [1])
m4_define([spin_minor_version], [0])
m4_define([spin_micro_version], [1])

m4_define([spin_release_status],
          [m4_if(m4_eval(spin_micro_version % 2), [1], [git],
                 [m4_if(m4_eval(spin_minor_version % 2), [1], [snapshot],
                                                                 [release])])])

m4_define([spin_version], [spin_major_version.spin_minor_version.spin_micro_version])

# change this only when breaking the API
m4_define([spin_api_version], [1.0])

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

#AC_PREREQ([2.63])
AC_INIT(SPINFRAMEWORK,
        [spin_version],
        [http://code.sat.qc.ca/redmine/projects/spinframework/issues/new],
        [spinframework],
        [http://www.spinframework.org])
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
  #AC_LANG_DEFINES_PROVIDED # remove autogen.sh warnings on OSX
  AM_CONDITIONAL([HAVE_OSX], true)
  AC_DEFINE([IS_OSX],[1],[Easy flag to tell us if this is OSX])
  LIBTOOL="glibtool"
  SRCDIR=`pwd`
  #LIBTOOL="libtool"
  CPPFLAGS="$CPPFLAGS -D__Darwin"
  #CPPFLAGS="$CPPFLAGS -D__Darwin -fpermissive"
  #CXXFLAGS=" -std=c++0x -stdlib=libc++"
  #CXX="clang++"
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
  #PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH:$SRCDIR/pkgconfig/osx
  #export PKG_CONFIG_PATH
  ;;
*)
  AM_CONDITIONAL([HAVE_OSX], false)
  ;;
esac

# On OSX, /usr/local/lib/pkgconfig is not included, and the 64-bit linux path is
# also often missing. We also provide some local .pc files since binary packages
# of flann and OpenNI (downloaded from www.pointclouds.org) do not include .pc
# files
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/local/lib64/pkgconfig:$PKG_CONFIG_PATH:$PWD/pkgconfig


CXXFLAGS+=" -Wall -Wextra -Wfatal-errors "
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

case "$host" in *-darwin* | *-macos*)
TINYXML_LIBS=-ltinyxml
;;
*)
PKG_CHECK_MODULES(TINYXML, tinyxml >= 2.0.0, [], AC_MSG_ERROR(TinyXML 2.0.0 or greater is required. Please see README))
;;
esac
AC_SUBST([TINYXML_LIBS])

# Method to disable Python (note the negative: DISable)
AC_ARG_ENABLE([python],
    AS_HELP_STRING([--enable-python], [Enables the Python interpreter within SPIN (enabled by default)]),
    [enable_python=$enableval],
    [enable_python=true])
if test "x$enable_python" != xtrue; then
AC_DEFINE([DISABLE_PYTHON],[1],[Disables python interpreter and thus all behaviours scripts in SPIN]) AC_MSG_NOTICE([Disabled Python scripts])
fi
AM_CONDITIONAL(HAVE_PYTHON, test "x$enable_python" = xtrue)

if test "x$enable_python" = xtrue; then

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
fi

#BOOST_REGEX
#BOOST_SYSTEM
#BOOST_FILESYSTEM
#BOOST_THREAD
#BOOST_PROGRAM_OPTIONS
if test "x$enable_python" = xtrue; then
BOOST_REQUIRE([1.35])
BOOST_PYTHON
fi


PKG_CHECK_MODULES(OSG, openscenegraph >= 2.8.0, , AC_MSG_ERROR([missing OpenSceneGraph]))

PKG_CHECK_MODULES(CPPINTROSPECTION, cppintrospection-3.0, , AC_MSG_ERROR([missing cppintrospection]))

PKG_CHECK_MODULES(OPENTHREADS, openthreads >= 2.4.0, , AC_MSG_ERROR([missing OpenThreads]))

PKG_CHECK_MODULES(LIBLO, liblo, , AC_MSG_ERROR([missing liblo]))

#PKG_CHECK_MODULES(SHAREDVIDEO, shared-video-0.6, 
#        [AC_DEFINE([WITH_SHARED_VIDEO],[],
#            [A flag that compiles additional features for use with the shared_video library from Scenic]) AC_MSG_NOTICE([Using scenic shared video library])], 
#        [AC_MSG_WARN([missing Scenic shared_video library])])


AC_ARG_ENABLE([sharedvideo], AS_HELP_STRING([--disable-sharedvideo], [Disable shmdata library]))
AS_IF([test "x$enable_sharedvideo" != "xno"], [
  PKG_CHECK_MODULES(SHAREDVIDEO, shmdata-osg-0.8 shmdata-any-0.8, have_sharedvideo=true, have_sharedvideo=false)
  if test "x${have_sharedvideo}" = "xfalse" ; then
    AC_MSG_WARN([missing package: libshmdata-0.8-dev ])
  else
    AC_DEFINE([WITH_SHAREDVIDEO],[],[A flag that compiles additional features for use with the shmdata library]) AC_MSG_NOTICE([Using shmdata library for sharing video streams])
    AC_MSG_NOTICE([SHAREDVIDEO_CFLAGS:    ${SHAREDVIDEO_CFLAGS}])
    AC_MSG_NOTICE([SHAREDVIDEO_LIBS:      ${SHAREDVIDEO_LIBS}])
  fi
])
AM_CONDITIONAL(SHAREDVIDEO, test -n "$SHAREDVIDEO_LIBS")

AC_CHECK_LIB([PocoFoundation],[main],[have_poco=true] [POCO_LIBS="-lPocoUtil -lPocoNet -lPocoXML -lPocoFoundation"], AC_MSG_WARN([missing Poco]))
if test "x${have_poco}" = "xtrue" ; then
    AC_DEFINE([WITH_POCO],[1],[Enables linking with the Poco and provides web services for SPIN]) AC_MSG_NOTICE([Enabled Poco for SPIN web services])
    AM_CONDITIONAL(HAVE_POCO, [true])
    AC_SUBST(POCO_LIBS)
else
    AM_CONDITIONAL(HAVE_POCO, [false])
    have_poco=false
fi

PKG_CHECK_MODULES(SPATOSC, spatosc-0.6, have_spatosc=true, have_spatosc=false)
if test "x${have_spatosc}" = "xfalse" ; then
    AC_MSG_WARN([missing package: libspatosc-dev])
else
    AC_DEFINE([WITH_SPATOSC],[1],[Enables linking with the SpatOSC library so that compatible audio renderers can be used for 3D sound spatialization]) AC_MSG_NOTICE([Enabled SpatOSC for audio])
    AC_MSG_NOTICE([SPATOSC_CFLAGS:        ${SPATOSC_CFLAGS}])
    AC_MSG_NOTICE([SPATOSC_LIBS:          ${SPATOSC_LIBS}])
fi

PKG_CHECK_MODULES(BULLET, [bullet >= 0.26], have_bullet=true, have_bullet=false)
if test "x${have_bullet}" = "xfalse" ; then
    AC_MSG_WARN([missing package: bullet])
else
    AC_DEFINE([WITH_BULLET],[1],[Enables linking with the Bullet physics library to provide collisions and rigid body dynamics]) AC_MSG_NOTICE([Enabled Bullet physics])
    AC_MSG_NOTICE([BULLET_CFLAGS:        ${BULLET_CFLAGS}])
    AC_MSG_NOTICE([BULLET_LIBS:          ${BULLET_LIBS}])
fi
AM_CONDITIONAL(BULLET, test -n "$BULLET_LIBS")

dnl PCL
PCL_PKGS="pcl_common-1.7 pcl_filters-1.7 pcl_io-1.7 pcl_kdtree-1.7 pcl_octree-1.7 pcl_sample_consensus-1.7 pcl_search-1.7"
PKG_CHECK_MODULES(PCL, ${PCL_PKGS}, have_pcl=true, have_pcl=false)
if test "x${have_pcl}" = "xfalse" ; then
    AC_MSG_WARN([missing packages: ${PCL_PKGS} (PointCloudLibrary)])
else
    AC_DEFINE([WITH_PCL],[1],[Enables linking with the Point Cloud Library (see www.pointclouds.org)]) AC_MSG_NOTICE([Enabled PointClouds using PCL])
    AC_MSG_NOTICE([PCL_CFLAGS:           ${PCL_CFLAGS}])
    AC_MSG_NOTICE([PCL_LIBS:             ${PCL_LIBS}])
fi
AM_CONDITIONAL(PCL, test -n "$PCL_LIBS")


#AC_CHECK_LIB([spnav],[spnav_open],[HAVE_SPNAV=1] [SPNAV_LIBS="-lspnav"],AC_MSG_WARN([missing package libspnav-dev]))
AC_CHECK_HEADERS([spnav.h],[have_spnav=true] [SPNAV_LIBS="-lspnav"],AC_MSG_WARN([missing package libspnav-dev]))
AM_CONDITIONAL(SPNAV, [test x$have_spnav = xtrue])
AC_SUBST(SPNAV_LIBS)


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

#        src/tinyxml/Makefile 
AC_CONFIG_FILES([
        Makefile 
        doxyfile
        src/Makefile 
        src/osgPPU/Makefile 
        src/osgWrappers/generator/Makefile 
        src/osgWrappers/introspection/Makefile 
        src/spin/Makefile
        src/Resources/Makefile
        src/Resources/images/Makefile
        src/Resources/models/Makefile
        src/Resources/desktopentries/Makefile
        src/Resources/camconfig/Makefile
        src/Resources/fonts/Makefile
        src/Resources/scripts/Makefile
        src/Resources/shaders/Makefile
        src/editor/Makefile
        src/lodifier/Makefile
        man/Makefile 
        tests/Makefile
        spinframework-${SPIN_API_VERSION}.pc:spin.pc.in
        completion/Makefile
        ])

AC_OUTPUT

[echo ""]
[echo ""]
[echo "======================================="]
[echo "Configured spinframework v${SPIN_VERSION}"]
[echo ""]
[echo "Python interpreter?        ${enable_python}"]
[echo "SpatOSC integration?       ${have_spatosc}"]
[echo "libSharedVideo?            ${have_sharedvideo}"]
[echo "PointCloud support?        ${have_pcl}"]
[echo "Poco web server?           ${have_poco}"]
[echo "Bullet Physics?            ${have_bullet}"]
[echo "SpaceNavigator support?    ${have_spnav}"]
[echo ""]
