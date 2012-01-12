# "foreign" means not GNU, and is common for avoiding boring messages about files organized differently from what gnu expects
AUTOMAKE_OPTIONS = foreign

ACLOCAL_AMFLAGS = -I m4

# Order of compilation really matters:
SUBDIRS = \
    src \
    tests

man_MANS = spinserver.1 spinviewer.1 
MAN_TEXT_FILES = man-spinserver.txt man-spinviewer.txt
EXTRA_DIST = \
    LICENSE \
    README \
    NEWS \
    $(man_MANS) \
    $(MAN_TEXT_FILES)

MAINTAINERCLEANFILES = $(man_MANS)

# The man pages depend on the --help strings and the version number.
common_mandeps = $(top_srcdir)/configure.ac

# Depend on the source, not the binary; we don't need to regenerate the
# binary when any source file changes, only the main one.
# Use -o so that the `missing' program can infer the output file.
spinserver.1: $(top_srcdir)/src/spin/spinServer.cpp $(common_mandeps) $(top_srcdir)/man-spinserver.txt
	$(HELP2MAN) --include=man-spinserver.txt --name="server for the SPIN Framework" --no-info $(top_builddir)/src/spin/spinserver -o $@

spinviewer.1: $(top_srcdir)/src/spin/spinViewer.cpp $(common_mandeps) $(top_srcdir)/man-spinviewer.txt
	$(HELP2MAN) --include=man-spinviewer.txt --name="3D viewer for the SPIN Framework" --no-info $(top_builddir)/src/spin/spinviewer -o $@

wrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating osgWrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers wrappers

doc: htmldoc
# $(MAKE) -C src/spin doc

apps:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating OSX App Bundles <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/spin spinviewer.app

INCLUDES = src/tinyxml

html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi

# man pages:
# if HAVE_DOXYGEN
# dist_man_MANS = $(top_srcdir)/man/man3/*.3
# $(top_srcdir)/man/man3/*.3: doxyfile.stamp
# 
# doxyfile.stamp:
# 	$(DOXYGEN) doxyfile
# 	echo `date -R` > doxyfile.stamp
# 
# CLEANFILES = doxyfile.stamp
# 
# all-local: doxyfile.stamp
# clean-local:
# 	rm -rf $(top_builddir)/man
# 	rm -rf html
# endif 

# pkg-config file:
DISTCLEANFILES = spin.pc
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = spin-@SPIN_API_VERSION@.pc

