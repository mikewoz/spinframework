SUBDIRS = wxsmith

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml $(PYTHON_CPPFLAGS)

EXTRA_DIST = \
    spinWX.cbp \
    spinWX.workspace

if ENABLE_EDITOR
bin_PROGRAMS = spineditor
spineditor_SOURCES =  spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
		wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
		wxSpinTreeCtrl.cpp
spineditor_LDFLAGS = -lSPIN $(SPIN_LDFLAGS)

# FIXME: I doubt this works
spineditor_LDADD = ../spin/libSPIN.la
endif

