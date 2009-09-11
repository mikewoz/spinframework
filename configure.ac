#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.63])
AC_INIT([SPINFramework], [0.2], [mike@mikewoz.com])
AC_MSG_NOTICE([Configuring the SPIN Framework])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([include/ModelNode.h])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

AC_CONFIG_FILES([Makefile src/genwrapper/Makefile src/osgWrappers/Makefile src/spin/Makefile])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_MAKE_SET # for recursive makefiles in subdirs
AC_PROG_INSTALL
AC_PROG_RANLIB # for making libraries
AC_PROG_LIBTOOL


# Checks for libraries.
#AM_PATH_XML2(2.4.0)
PKG_CHECK_MODULES(XML, libxml-2.0 >= 2.4)

AX_BOOST_BASE([1.35])
AX_BOOST_REGEX
#  if test "x$ax_cv_boost_regex" = "xno"; then
#    echo
#    echo "Error: You need to install the boost regex library!"
#    echo
#    exit
#fi
 

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h stddef.h stdlib.h string.h sys/time.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_CHECK_TYPES([ptrdiff_t])

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([gettimeofday memmove mkdir setenv sqrt strchr strpbrk uname])

#AC_OUTPUT(Makefile src/genwrapper/Makefile src/osgWrappers/Makefile src/spin/Makefile)
AC_OUTPUT
