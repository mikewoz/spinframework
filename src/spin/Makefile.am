AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I$(PYTHON_INCLUDE_DIR)


#INCLUDES = -I$(top_srcdir)/src/tinyxml

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libSPIN.la libSPINPyWrap.la

libSPIN_la_CPPFLAGS = \
	$(AM_CPPFLAGS)


libSPIN_la_LIBADD = \
	$(top_srcdir)/src/osgWrappers/libSPINwrappers.la \
	$(top_srcdir)/src/tinyxml/libtinyxml.la

#$(top_srcdir)/src/osgWrappers/libSPINwrappers.a \
#$(top_srcdir)/src/tinyxml/libtinyxml.a

libSPIN_la_LDFLAGS = \
	$(BOOST_LDFLAGS) \
	$(BOOST_FILESYSTEM_LIB) \
	-lOpenThreads -losg \
	-losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil \
	-losgFX -losgParticle -losgViewer \
	$(OSGINTROSPECTION_LIB) $(LIBLO_LDFLAGS) -l$(PYTHON_LIB) -l$(BOOST_PYTHON_LIB) 

libSPIN_la_SOURCES = \
	spinContext.cpp osgUtil.cpp spinUtil.cpp libloUtil.cpp \
	SceneManager.cpp MediaManager.cpp ViewerManipulator.cpp \
	ReferencedNode.cpp ReferencedState.cpp \
	UserNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
	GridNode.cpp LightSource.cpp RayNode.cpp Contour.cpp \
	ConstraintsNode.cpp PointerNode.cpp MeasurementNode.cpp \
	AnimationNode.cpp SwitchNode.cpp TextNode.cpp \
	DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp SoundConnection.cpp \
	VideoTexture.cpp SharedVideoTexture.cpp nodeVisitors.cpp

libSPINPyWrap_la_CPPFLAGS = $(AM_CPPFLAGS)
libSPINPyWrap_la_LIBADD = libSPIN.la
libSPINPyWrap_la_LDFLAGS = $(BOOST_LDFLAGS) -losg $(OSGINTROSPECTION_LIB) -l$(PYTHON_LIB) -l$(BOOST_PYTHON_LIB) 
libSPINPyWrap_la_SOURCES = python_wrappers.cpp

bin_PROGRAMS = spinServer spinViewer
#bin_PROGRAMS = spinServer spinViewer spin

#AM_LDFLAGS = -llo

#  OSG_LIBS += -lc -lm -ldl -lGL -lX11 -lpthread \
#  -lOpenThreads -losg -losgGA -losgSim -losgDB -losgIntrospection \
# -losgManipulator -losgText -losgUtil -losgFX -losgParticle -losgViewer

spinServer_SOURCES = spinServer.cpp
spinServer_CPPFLAGS = $(AM_CPPFLAGS)
#spinServer_LDADD = $(top_srcdir)/src/spin/libSPIN.so
#spinServer_LDADD = -lSPIN
spinServer_LDADD = $(top_srcdir)/src/spin/libSPIN.la
spinServer_LDFLAGS = -l$(PYTHON_LIB) $(BOOST_LDFLAGS) -l$(BOOST_PYTHON_LIB) 
#spinServer_LDADD = -lSPIN -lOpenThreads -losg -losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil -losgFX -losgParticle -losgViewer $(OSGINTROSPECTION_LIB) $(LIBLO_LIB)

spinViewer_SOURCES = spinViewer.cpp
#spinViewer_LDADD = $(top_srcdir)/src/spin/libSPIN.so
spinViewer_LDFLAGS = -lSPIN


scriptsdir = $(datadir)/spinFramework/scripts
scripts_DATA = scripts/hello.py scripts/spin.py

#spin_SOURCES = spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
#      wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
#      wxSpinTreeCtrl.cpp
#spin_LDADD = libSPIN



