BUILT_SOURCES = \
        ReferencedNode.cpp ReferencedState.cpp \
        UserNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
        GridNode.cpp LightSource.cpp RayNode.cpp Contour.cpp \
        ConstraintsNode.cpp PointerNode.cpp MeasurementNode.cpp \
        AnimationNode.cpp SwitchNode.cpp TextNode.cpp \
        DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp SoundConnection.cpp \
        Texture.cpp VideoTexture.cpp SharedVideoTexture.cpp 

%.cpp: %.h


$(BUILT_SOURCES):
	-rm $(top_srcdir)/include/*.h~
	cd $(top_srcdir) && src/genwrapper/genwrapper -d . doxygen | doxygen - && cd -
	cd $(top_srcdir) && src/genwrapper/genwrapper -v QUIET -c genwrapper.conf doxygen . && cd -

# use noinst_LIBRARIES to create a library that won't be installed
noinst_LTLIBRARIES = libSPINwrappers.la
#noinst_LIBRARIES = libSPINwrappers.la

libSPINwrappers_la_SOURCES = $(BUILT_SOURCES)

#        ReferencedNode.cpp ReferencedState.cpp \
#        UserNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
#        GridNode.cpp LightSource.cpp RayNode.cpp Contour.cpp \
#        ConstraintsNode.cpp PointerNode.cpp MeasurementNode.cpp \
#        AnimationNode.cpp SwitchNode.cpp TextNode.cpp \
#        DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp SoundConnection.cpp \
#        VideoTexture.cpp SharedVideoTexture.cpp 




libSPINwrappers_la_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(top_srcdir)/src/tinyxml $(PYTHON_CPPFLAGS)

