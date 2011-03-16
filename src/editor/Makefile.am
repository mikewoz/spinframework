AM_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(LIBLO_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(WX_CPPFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/include/spineditor \
    -I$(top_srcdir)/src/tinyxml

#  -arch x86_64

EXTRA_DIST = \
    layout.cbp

if ENABLE_EDITOR

bin_PROGRAMS = spineditor

spineditor_SOURCES = \
    app.cpp \
    app.h \
    GUIFrame.cpp \
    GUIFrame.h \
    main.cpp \
    main.h \
    unused.h

spineditor_LDFLAGS = \
    $(BOOST_FILESYSTEM_LIB) \
    $(BOOST_LDFLAGS) \
    $(BOOST_THREAD_LIB) \
    $(CPPINTROSPECTION_LIBS) \
    $(LIBLO_LIBS) \
    $(OPENTHREADS_LIBS) \
    $(OSG_LIBS) \
    $(PYTHON_LDFLAGS) \
    $(SHARED_VIDEO_LIB) \
    -l$(BOOST_PYTHON_LIB) \
    $(WX_LIBS)

# dnl     CPPFLAGS="$CPPFLAGS $WX_CPPFLAGS"
# dnl     CXXFLAGS="$CXXFLAGS $WX_CXXFLAGS_ONLY"
# dnl     CFLAGS="$CFLAGS $WX_CFLAGS_ONLY"

# FIXME: I doubt this works
spineditor_LDADD = ../spin/libSPIN.la

endif

