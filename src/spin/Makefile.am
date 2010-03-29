AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml $(PYTHON_CPPFLAGS)



# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libSPIN.la libSPINPyWrap.la

#libSPIN_la_CPPFLAGS = $(SPIN_CPPFLAGS)

libSPIN_la_LIBADD = \
	$(top_srcdir)/src/osgWrappers/libSPINwrappers.la \
	$(top_srcdir)/src/tinyxml/libtinyxml.la

libSPIN_la_LDFLAGS = \
	$(PYTHON_LDFLAGS) \
	$(BOOST_LDFLAGS) \
	$(BOOST_FILESYSTEM_LIB) \
	-l$(BOOST_PYTHON_LIB) \
	-lOpenThreads -losg \
	-losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil \
	-losgFX -losgParticle -losgViewer \
	$(OSGINTROSPECTION_LIB) $(LIBLO_LDFLAGS)

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

libSPINPyWrap_la_SOURCES = python_wrappers.cpp
libSPINPyWrap_la_LDFLAGS = -shrext ".so" # python expects .so on OSX
libSPINPyWrap_la_LIBADD = libSPIN.la

bin_PROGRAMS = spinServer spinViewer spinDocGenerator

spinServer_SOURCES = spinServer.cpp
spinServer_LDFLAGS = -lSPIN # $(SPIN_LDFLAGS)

spinViewer_SOURCES = spinViewer.cpp
spinViewer_LDFLAGS = -lSPIN # $(SPIN_LDFLAGS)

spinDocGenerator_SOURCES = docGenerator.cpp
spinDocGenerator_LDFLAGS = -lSPIN $(SPIN_LDFLAGS)

scriptsdir = $(datadir)/spinFramework/scripts
scripts_DATA = scripts/hello.py scripts/spin.py

#spin_SOURCES = spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
#      wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
#      wxSpinTreeCtrl.cpp
#spin_LDADD = libSPIN


doc:
	./docGenerator
	-cp *.html ../../doxygen/html
