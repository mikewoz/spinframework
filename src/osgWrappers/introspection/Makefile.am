#BUILT_SOURCES = \
#        ReferencedNode.cpp ReferencedStateSet.cpp \
#        UserNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
#        GridNode.cpp LightSource.cpp RayNode.cpp Contour.cpp \
#        ConstraintsNode.cpp PointerNode.cpp MeasurementNode.cpp \
#        AnimationNode.cpp SwitchNode.cpp TextNode.cpp \
#        DSPNode.cpp SoundNode.cpp SoundSpace.cpp Listener.cpp SoundConnection.cpp \
#        ImageTexture.cpp VideoTexture.cpp SharedVideoTexture.cpp 
#CLEANFILES=$(BUILT_SOURCES)
#%.cpp: %.h

#FIXME: These depend on the headers, and more specifically, each generated wrapper should only depend on one header
#$(BUILT_SOURCES):
#	-rm -f $(top_srcdir)/include/*.h~
#	cd $(top_srcdir) && src/genwrapper/genwrapper -d . doxygen | doxygen - && cd -
#	cd $(top_srcdir) && src/genwrapper/genwrapper -v QUIET -c genwrapper.conf doxygen . && cd -

# use noinst_LIBRARIES to create a library that won't be installed
noinst_LTLIBRARIES = libSPINwrappers.la

#libSPINwrappers_la_SOURCES = #$(BUILT_SOURCES)

# please keep those in alphabetical order:
libSPINwrappers_la_SOURCES = \
	ReferencedNode.cpp \
    AnimationNode.cpp \
    AttractorNode.cpp \
    ConstraintsNode.cpp \
    Contour.cpp \
    DSPNode.cpp \
    Fog.cpp \
    GridNode.cpp \
    GroupNode.cpp \
    ImageTexture.cpp \
    LightSource.cpp \
    Listener.cpp \
    MeasurementNode.cpp \
    Menu3D.cpp \
    ModelNode.cpp \
    PointerNode.cpp \
    RayNode.cpp \
    ReferencedStateSet.cpp \
    ReporterNode.cpp \
    ShapeNode.cpp \
    SharedVideoTexture.cpp \
    SoundConnection.cpp \
    SoundNode.cpp \
    SoundSpace.cpp \
    SwitchNode.cpp \
    TextNode.cpp \
    UserNode.cpp \
    VideoTexture.cpp

#    $(top_srcdir)/include/*.h \
#    $(top_srcdir)/src/spin/*.h \
#    $(top_srcdir)/src/tinyxml/*.h

libSPINwrappers_la_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src/spin \
    -I$(top_srcdir)/src/tinyxml

