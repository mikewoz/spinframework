# use noinst_LIBRARIES to create a library that won't be installed
noinst_LTLIBRARIES = libSPINwrappers.la

# please keep those in alphabetical order:
# We only need to wrap the childs of ReferencedNode and ReferencedStateSet:
libSPINwrappers_la_SOURCES = \
    AnimationNode.cpp \
    AttractorNode.cpp \
    ConstraintsNode.cpp \
    Contour.cpp \
    DSPNode.cpp \
    Fog.cpp \
    GeometryNode.cpp \
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
    ReferencedNode.cpp \
    ReferencedStateSet.cpp \
    ReporterNode.cpp \
    Shader.cpp \
    ShapeNode.cpp \
    SharedVideoTexture.cpp \
    SoundConnection.cpp \
    SoundNode.cpp \
    SoundSpace.cpp \
    SwitchNode.cpp \
    TextNode.cpp \
    UserNode.cpp \
    VideoTexture.cpp

libSPINwrappers_la_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(SPATOSC_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src/spin \
    -I$(top_srcdir)/src/tinyxml

# TODO: Could be nice to regenerate the wrapper every time a source file from src/spin changes:
# FIXME: These depend on the headers, and more specifically, each generated wrapper should only depend on one header
#$(BUILT_SOURCES):
#	-rm -f $(top_srcdir)/include/*.h~
#	cd $(top_srcdir) && src/genwrapper/genwrapper -d . doxygen | doxygen - && cd -
#	cd $(top_srcdir) && src/genwrapper/genwrapper -v QUIET -c genwrapper.conf doxygen . && cd -
