AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I$(PYTHON_INCLUDE_DIR)



#INCLUDES = -I$(top_srcdir)/src/tinyxml

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libSPIN.la libSPINPyWrap.la

libSPIN_la_CPPFLAGS = \
	$(AM_CPPFLAGS)


#-I$(top_srcdir)/src/tinyxml \
#-I$(BOOST_CPPFLAGS) \
#-I$(PYTHON_INCLUDE_DIR)


libSPIN_la_LIBADD = \
	$(top_srcdir)/src/osgWrappers/libSPINwrappers.la \
	$(top_srcdir)/src/tinyxml/libtinyxml.la

libSPIN_la_LDFLAGS = \
	$(BOOST_LDFLAGS) \
	$(BOOST_FILESYSTEM_LIB) \
	$(BOOST_PYTHON_LIB) \
	-lOpenThreads -losg \
	-losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil \
	-losgFX -losgParticle -losgViewer \
	$(OSGINTROSPECTION_LIB) $(LIBLO_LDFLAGS) \
	-l$(PYTHON_LIB)

# -l$(BOOST_PYTHON_LIB) 

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

libSPINPyWrap_la_CPPFLAGS = $(AM_CPPFLAGS) -I$(PYTHON_INCLUDE_DIR) -I$(top_srcdir)/src/tinyxml
libSPINPyWrap_la_LIBADD = libSPIN.la
libSPINPyWrap_la_LDFLAGS = $(BOOST_LDFLAGS) -losg $(OSGINTROSPECTION_LIB) -l$(PYTHON_LIB) $(BOOST_PYTHON_LIB) 
libSPINPyWrap_la_SOURCES = python_wrappers.cpp

bin_PROGRAMS = spinServer spinViewer spinDocGenerator
#bin_PROGRAMS = spinServer spinViewer spin

#AM_LDFLAGS = -llo

#  OSG_LIBS += -lc -lm -ldl -lGL -lX11 -lpthread \
#  -lOpenThreads -losg -losgGA -losgSim -losgDB -losgIntrospection \
# -losgManipulator -losgText -losgUtil -losgFX -losgParticle -losgViewer

spinServer_SOURCES = spinServer.cpp
spinServer_LDFLAGS = -lSPIN -l$(PYTHON_LIB) $(BOOST_LDFLAGS) $(BOOST_PYTHON_LIB) 

spinViewer_SOURCES = spinViewer.cpp
spinViewer_LDFLAGS = -lSPIN -l$(PYTHON_LIB) $(BOOST_LDFLAGS) $(BOOST_PYTHON_LIB) 

spinDocGenerator_SOURCES = docGenerator.cpp
spinDocGenerator_LDFLAGS = -lSPIN -l$(PYTHON_LIB) $(BOOST_LDFLAGS) $(BOOST_PYTHON_LIB) 

scriptsdir = $(datadir)/spinFramework/scripts
scripts_DATA = scripts/hello.py scripts/spin.py

#spin_SOURCES = spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
#      wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
#      wxSpinTreeCtrl.cpp
#spin_LDADD = libSPIN


doc:
	./docGenerator
	-cp *.html ../../doxygen/html
