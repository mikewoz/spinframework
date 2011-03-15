AM_CPPFLAGS = \
    -I$(top_srcdir)/include/spineditor \
    -I$(top_srcdir)/src/tinyxml \
    $(PYTHON_CPPFLAGS) \
    $(WX_CPPFLAGS) -arch x86_64

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
    main.h

spineditor_LDFLAGS = -lSPIN $(SPIN_LDFLAGS) $(WX_LIBS) -arch x86_64

# dnl     CPPFLAGS="$CPPFLAGS $WX_CPPFLAGS"
# dnl     CXXFLAGS="$CXXFLAGS $WX_CXXFLAGS_ONLY"
# dnl     CFLAGS="$CFLAGS $WX_CFLAGS_ONLY"

# FIXME: I doubt this works
spineditor_LDADD = ../spin/libSPIN.la

endif

