# use noinst_LIBRARIES to create a library that won't be installed
noinst_LTLIBRARIES = libSPINwrappers.la

# please keep those in alphabetical order:
# We only need to wrap the childs of ReferencedNode and ReferencedStateSet:
libSPINwrappers_la_SOURCES = \
    animationnode.cpp \
    attractornode.cpp \
    constraintsnode.cpp \
    contour.cpp \
    dspnode.cpp \
    fog.cpp \
    geometrynode.cpp \
    gridnode.cpp \
    groupnode.cpp \
    imagetexture.cpp \
    lightsource.cpp \
    listener.cpp \
    lodnode.cpp \
    menu3d.cpp \
    modelnode.cpp \
    particlesystem.cpp \
    pointernode.cpp \
    raynode.cpp \
    referencednode.cpp \
    referencedstateset.cpp \
    reporternode.cpp \
    shader.cpp \
    shapenode.cpp \
    sharedvideotexture.cpp \
    soundnode.cpp \
    soundspace.cpp \
    switchnode.cpp \
    textnode.cpp \
    usernode.cpp \
    videotexture.cpp

libSPINwrappers_la_CPPFLAGS = \
    -std=gnu++0x \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(SHAREDVIDEO_CFLAGS) \
    $(SPATOSC_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src/spin \
    -I$(top_srcdir)/src/tinyxml


if BULLET
libSPINwrappers_la_CPPFLAGS += $(BULLET_CFLAGS)
libSPINwrappers_la_SOURCES += collisionshape.cpp
endif

if PCL
libSPINwrappers_la_CPPFLAGS += $(PCL_CFLAGS) \
    -I/usr/include/ni
libSPINwrappers_la_SOURCES += pointcloud.cpp
endif


# TODO: Could be nice to regenerate the wrapper every time a source file from src/spin changes:
# FIXME: These depend on the headers, and more specifically, each generated wrapper should only depend on one header
#$(BUILT_SOURCES):
#	-rm -f $(top_srcdir)/include/*.h~
#	cd $(top_srcdir) && src/genwrapper/genwrapper -d . doxygen | doxygen - && cd -
#	cd $(top_srcdir) && src/genwrapper/genwrapper -v QUIET -c genwrapper.conf doxygen . && cd -
