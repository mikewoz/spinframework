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

if ENABLE_EDITOR

bin_PROGRAMS = spineditor

spineditor_SOURCES = \
    introspection.cpp \
    introspection.h \
    main.cpp \
	MainFrame.cpp \
	MainFrame.h \
	MainFrame_base.cpp \
	MainFrame_base.h \
	dialog_NewNode.cpp \
	dialog_NewNode.h \
	dialog_NewNode_base.cpp \
	dialog_NewNode_base.h \
	wxSpinTreeCtrl.cpp \
	wxSpinTreeCtrl.h \
	wxSpinTreeVisitor.cpp \
	wxSpinTreeVisitor.h \
    unused.h
	
#	wxSpinPropGrid.cpp \
#	wxSpinPropGrid.h 

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

spineditor_LDADD = ../spin/libSPIN.la

endif

