AM_CPPFLAGS = \
	$(BOOST_CPPFLAGS) \
	$(CPPINTROSPECTION_CFLAGS) \
	$(PYTHON_CPPFLAGS) \
    $(LIBLO_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) \
    $(SHAREDVIDEO_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src \
    -I$(top_srcdir)/src/tinyxml

LDADD = $(top_builddir)/src/spin/libspinframework-@SPIN_API_VERSION@.la

AM_LDFLAGS = \
	$(BOOST_FILESYSTEM_LIB) \
	$(BOOST_LDFLAGS) \
	$(CPPINTROSPECTION_LIBS) \
	$(PYTHON_LDFLAGS) \
	$(BOOST_PYTHON_LIB) \
    $(BOOST_THREAD_LIB) \
    $(LIBLO_LIBS) \
    $(OPENTHREADS_LIBS) \
    $(OSG_LIBS) \
    $(SHAREDVIDEO_LIBS)

check_PROGRAMS = \
    check_it_works

TESTS = $(check_PROGRAMS)

check_it_works_SOURCES = check_it_works.cpp

