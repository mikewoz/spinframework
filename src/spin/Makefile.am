
INCLUDES = -I$(top_srcdir)/src/tinyxml -I$(top_srcdir)/src/mivilleEngine

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libSPIN.la
libSPIN_la_LIBADD = ../osgWrappers/libSPINwrapper.a ../tinyxml/libtinyxml.a
libSPIN_la_SOURCES = spinContext.cpp osgUtil.cpp spinUtil.cpp libloUtil.cpp \
       SceneManager.cpp MediaManager.cpp ReferencedNode.cpp \
       GroupNode.cpp ShapeNode.cpp ModelNode.cpp VideoNode.cpp LightSource.cpp \
       RayNode.cpp Contour.cpp ConstraintsNode.cpp PointerNode.cpp \
       MeasurementNode.cpp UserNode.cpp \
       DSPNode.cpp SoundNode.cpp SoundSpace.cpp SoundConnection.cpp \
       SharedVideoNode.cpp sharedVideoBuffer.cpp

bin_PROGRAMS = spinServer spinViewer spin


spinServer_SOURCES = spinServer.cpp
spinServer_LDADD = libSPIN

spinViewer_SOURCES = spinViewer.cpp
spinViewer_LDADD = libSPIN


spin_SOURCES = spinWX.cpp wxSpinMain.cpp wxSpinEditor.cpp wxSpinRenderer.cpp \
      wxSpinPropGrid.cpp wxSpinSettings.cpp wxSpinTreeVisitor.cpp \
      wxSpinTreeCtrl.cpp
spin_LDADD = libSPIN





