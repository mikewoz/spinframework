# "foreign" means not GNU, and is common for avoiding boring messages about files organized differently from what gnu expects
AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4

# Order of compilation really matters:
SUBDIRS = \
    completion \
    src \
    man \
    tests

EXTRA_DIST = \
    buildWrappers.sh \
    LICENSE \
    README \
    NEWS

wrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating osgWrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers wrappers

doc: htmldoc
	$(MAKE) -C src/spin doc

apps:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating OSX App Bundles <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/spin spinviewer.app

html-local: htmldoc
htmldoc:
	@if [ -z `which doxygen` ]; then echo "No Doxygen!"; else doxygen; fi

# pkg-config file:
DISTCLEANFILES = spin.pc
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = spinframework-@SPIN_API_VERSION@.pc

