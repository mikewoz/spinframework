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

all-local: wrappers

wrappers:
	-rm $(top_srcdir)/include/*.h~
	cd $(top_srcdir) && src/genwrapper/genwrapper -d . doxygen | doxygen - && cd -
	cd $(top_srcdir) && src/genwrapper/genwrapper -v QUIET -c genwrapper.conf doxygen . && cd -

