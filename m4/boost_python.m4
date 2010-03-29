# ===========================================================================
#            http://autoconf-archive.cryp.to/ax_boost_python.html
# ===========================================================================
#
# SYNOPSIS
#
#   AX_BOOST_PYTHON
#
# DESCRIPTION
#
#   This macro checks to see if the Boost.Python library is installed. It
#   also attempts to guess the currect library name using several attempts.
#   It tries to build the library name using a user supplied name or suffix
#   and then just the raw library.
#
#   If the library is found, HAVE_BOOST_PYTHON is defined and
#   BOOST_PYTHON_LIB is set to the name of the library.
#
#   This macro calls AC_SUBST(BOOST_PYTHON_LIB).
#
#   In order to ensure that the Python headers are specified on the include
#   path, this macro requires AX_PYTHON to be called.
#
# LAST MODIFICATION
#
#   2008-04-12
#
# COPYLEFT
#
#   Copyright (c) 2008 Michael Tindal
#
#   This program is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation; either version 2 of the License, or (at your
#   option) any later version.
#
#   This program is distributed in the hope that it will be useful, but
#   WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#   Public License for more details.
#
#   You should have received a copy of the GNU General Public License along
#   with this program. If not, see <http://www.gnu.org/licenses/>.
#
#   As a special exception, the respective Autoconf Macro's copyright owner
#   gives unlimited permission to copy, distribute and modify the configure
#   scripts that are the output of Autoconf when processing the Macro. You
#   need not follow the terms of the GNU General Public License when using
#   or distributing such scripts, even though portions of the text of the
#   Macro appear in them. The GNU General Public License (GPL) does govern
#   all other use of the material that constitutes the Autoconf Macro.
#
#   This special exception to the GPL applies to versions of the Autoconf
#   Macro released by the Autoconf Macro Archive. When you make and
#   distribute a modified version of the Autoconf Macro, you may extend this
#   special exception to the GPL to apply to your modified version as well.

AC_DEFUN([AX_BOOST_PYTHON],
[
	AC_ARG_WITH([boost-python],
	AS_HELP_STRING([--with-boost-python@<:@=special-lib@:>@],
                   [use the Python library from boost - it is possible to specify a certain library for the linker
                        e.g. --with-boost-python=boost_python-gcc-mt-d-1_33_1 ]),
        [
        if test "$withval" = "no"; then
			want_boost="no"
        elif test "$withval" = "yes"; then
            want_boost="yes"
            ax_boost_user_python_lib=""
        else
		    want_boost="yes"
		ax_boost_user_python_lib="$withval"
		fi
        ],
        [want_boost="yes"]
	)

	if test "x$want_boost" = "xyes"; then
		AC_REQUIRE([AX_PYTHON])
        AC_REQUIRE([AC_PROG_CC])
		CPPFLAGS_SAVED="$CPPFLAGS"
		CPPFLAGS="$CPPFLAGS $BOOST_CPPFLAGS"
		if test x$PYTHON_INCLUDE_DIR != x; then
			CPPFLAGS="-I$PYTHON_INCLUDE_DIR $CPPFLAGS"
		fi
		export CPPFLAGS

		LDFLAGS_SAVED="$LDFLAGS"
		LDFLAGS="$LDFLAGS $BOOST_LDFLAGS"
		export LDFLAGS

        AC_CACHE_CHECK(whether the Boost::Python library is available,
					   ax_cv_boost_python,
        [AC_LANG_PUSH([C++])
			 AC_COMPILE_IFELSE(AC_LANG_PROGRAM([[@%:@include <boost/python.hpp>
												]],
                                   [[boost::python::str("");]]),
                   ax_cv_boost_python=yes, ax_cv_boost_python=no)
         AC_LANG_POP([C++])
		])
		if test "x$ax_cv_boost_python" = "xyes"; then
			AC_DEFINE(HAVE_BOOST_PYTHON,,[define if the Boost::Python library is available])
            BOOSTLIBDIR=`echo $BOOST_LDFLAGS | sed -e 's/@<:@^\/@:>@*//'`
            if test "x$ax_boost_user_python_lib" = "x"; then
				for end in "-mt.so" "-mt-py$PYTHON_VERSION_SHORT.so" "-mt-py$PYTHON_VERSION_SHORT.so.*" "-gcc*.so.*"; do
					for lib in `find $BOOSTLIBDIR/libboost_python$end -nowarn 2>&-| sort -nr`; do
						ax_lib=`echo  -n ${lib} | perl -p -e 's#.*lib(boost_python.*)\.(?:so|a).*#@S|@1#'`
					    AC_CHECK_LIB($ax_lib, exit,
							[BOOST_PYTHON_LIB="-l$ax_lib"; AC_SUBST(BOOST_PYTHON_LIB) link_python="yes"; break 2],
							[link_python="no"], -l$PYTHON_LIB)
					done
				done
                if test "x$link_python" != "xyes"; then
                for libextension in `ls $BOOSTLIBDIR/boost_python*.{dll,a}* 2>/dev/null | sed 's,.*/,,' | sed -e 's;^\(boost_python.*\)\.dll.*$;\1;' -e 's;^\(boost_python.*\)\.a*$;\1;'` ; do
                     ax_lib=${libextension}
				    AC_CHECK_LIB($ax_lib, exit,
                                 [BOOST_PYTHON_LIB="-l$ax_lib"; AC_SUBST(BOOST_PYTHON_LIB) link_python="yes"; break],
                                 [link_python="no"], -l$PYTHON_LIB)
				done
                fi

            else
               for ax_lib in $ax_boost_user_python_lib boost_python-$ax_boost_user_python_lib; do
				      AC_CHECK_LIB($ax_lib, main,
                                   [BOOST_PYTHON_LIB="-l$ax_lib"; AC_SUBST(BOOST_PYTHON_LIB) link_python="yes"; break],
                                   [link_python="no"], -l$PYTHON_LIB)
               done
            fi
			if test "x$link_python" != "xyes"; then
				AC_MSG_ERROR(Could not link against $ax_lib !)
			fi
		fi

		CPPFLAGS="$CPPFLAGS_SAVED"
	LDFLAGS="$LDFLAGS_SAVED"
	fi
])
