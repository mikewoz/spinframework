AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml

# use noinst_LIBRARIES to create a library that won't be installed
noinst_LTLIBRARIES = libSPINwrappers.la
#noinst_LIBRARIES = libSPINwrappers.la

libSPINwrappers_la_SOURCES = \
	ReferencedNode.cpp ReferencedState.cpp \
	UserNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
	GridNode.cpp LightSource.cpp RayNode.cpp Contour.cpp \
	ConstraintsNode.cpp PointerNode.cpp MeasurementNode.cpp \
	AnimationNode.cpp SwitchNode.cpp TextNode.cpp \
	DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp SoundConnection.cpp \
	VideoTexture.cpp SharedVideoTexture.cpp 

GENWRAPPER_BIN= $(top_srcdir)/src/genwrapper/genwrapper

wrappers:
	-rm $(top_srcdir)/include/*.h~
	$(GENWRAPPER_BIN) -d . doxygen | doxygen -
	$(GENWRAPPER_BIN) -v QUIET -c genwrapper.conf doxygen .

