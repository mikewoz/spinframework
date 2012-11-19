man_MANS = spinserver.1 spinviewer.1 spindocgenerator.1 lodifier.1
MAN_TEXT_FILES = man-spinserver.txt man-spinviewer.txt man-lodifier.txt

EXTRA_DIST = \
    $(man_MANS) \
    $(MAN_TEXT_FILES)

MAINTAINERCLEANFILES = $(man_MANS)

# The man pages depend on the --help strings and the version number.
common_mandeps = $(top_srcdir)/configure.ac

# Depend on the source, not the binary; we don't need to regenerate the
# binary when any source file changes, only the main one.
# Use -o so that the `missing' program can infer the output file.
spinserver.1: $(top_srcdir)/src/spin/spinServer.cpp $(common_mandeps) $(top_srcdir)/man/man-spinserver.txt
	$(HELP2MAN) --include=man-spinserver.txt --name="server for the SPIN Framework" --no-info $(top_builddir)/src/spin/spinserver -o $@

spinviewer.1: $(top_srcdir)/src/spin/spinViewer.cpp $(common_mandeps) $(top_srcdir)/man/man-spinviewer.txt
	$(HELP2MAN) --include=man-spinviewer.txt --name="3D viewer for the SPIN Framework" --no-info $(top_builddir)/src/spin/spinviewer -o $@

lodifier.1: $(top_srcdir)/src/lodifier/lodifier.cpp $(common_mandeps) $(top_srcdir)/man/man-lodifier.txt
	$(HELP2MAN) --include=man-lodifier.txt --name="Automatic level-of-detail generator for 3d scenes" --no-info $(top_builddir)/src/lodifier/lodifier -o $@



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

