AM_CPPFLAGS = \
    -I$(top_srcdir)/include/spineditor \
    -I$(top_srcdir)/src/tinyxml \
    $(PYTHON_CPPFLAGS) \
    $(WX_CPPFLAGS)

EXTRA_DIST = \
    layout.cbp

if ENABLE_EDITOR

bin_PROGRAMS = spineditor

spineditor_SOURCES = \
    editor/app.cpp \
    editor/app.h \
    editor/GUIFrame.cpp \
    editor/GUIFrame.h \
    editor/main.cpp \
    editor/main.h

spineditor_LDFLAGS = -lSPIN $(SPIN_LDFLAGS) $(WX_LIBS)

# dnl     CPPFLAGS="$CPPFLAGS $WX_CPPFLAGS"
# dnl     CXXFLAGS="$CXXFLAGS $WX_CXXFLAGS_ONLY"
# dnl     CFLAGS="$CFLAGS $WX_CFLAGS_ONLY"

# FIXME: I doubt this works
spineditor_LDADD = ../spin/libSPIN.la

endif

