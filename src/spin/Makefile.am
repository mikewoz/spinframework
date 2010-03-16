
INCLUDES = -I$(top_srcdir)/src/tinyxml

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libSPIN.la

libSPIN_la_LIBADD = \
	$(top_srcdir)/src/osgWrappers/libSPINwrappers.a \
	$(top_srcdir)/src/tinyxml/libtinyxml.a

libSPIN_la_LDFLAGS = \
	$(BOOST_LDFLAGS) \
	-lOpenThreads -losg \
	-losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil \
	-losgFX -losgParticle -losgViewer \
	$(OSGINTROSPECTION_LIB) $(LIBLO_LDFLAGS)

libSPIN_la_SOURCES = \
	spinContext.cpp osgUtil.cpp spinUtil.cpp libloUtil.cpp \
	SceneManager.cpp MediaManager.cpp ViewerManipulator.cpp \
	ReferencedNode.cpp ReferencedState.cpp \
	GroupNode.cpp ShapeNode.cpp ModelNode.cpp LightSource.cpp \
	RayNode.cpp Contour.cpp ConstraintsNode.cpp PointerNode.cpp \
	AnimationNode.cpp MeasurementNode.cpp UserNode.cpp \
	GridNode.cpp SwitchNode.cpp TextNode.cpp VideoTexture.cpp \
	DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp \
	SoundConnection.cpp 
       #SharedVideoNode.cpp sharedVideoBuffer.cpp


bin_PROGRAMS = spinServer spinViewer
#bin_PROGRAMS = spinServer spinViewer spin

#AM_LDFLAGS = -llo

#  OSG_LIBS += -lc -lm -ldl -lGL -lX11 -lpthread \
#  -lOpenThreads -losg -losgGA -losgSim -losgDB -losgIntrospection \
# -losgManipulator -losgText -losgUtil -losgFX -losgParticle -losgViewer

spinServer_SOURCES = spinServer.cpp
#spinServer_LDADD = $(top_srcdir)/src/spin/libSPIN.so
spinServer_LDADD = -lSPIN
#spinServer_LDADD = -lSPIN -lOpenThreads -losg -losgGA -losgSim -losgDB -losgManipulator -losgText -losgUtil -losgFX -losgParticle -losgViewer $(OSGINTROSPECTION_LIB) $(LIBLO_LIB)

spinViewer_SOURCES = spinViewer.cpp
#spinViewer_LDADD = $(top_srcdir)/src/spin/libSPIN.so
spinViewer_LDFLAGS = -lSPIN



#spin_SOURCES = spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
#      wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
#      wxSpinTreeCtrl.cpp
#spin_LDADD = libSPIN



