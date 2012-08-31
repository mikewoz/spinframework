AM_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(LIBLO_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) \
    $(SHAREDVIDEO_CFLAGS) \
    $(SPATOSC_CFLAGS) \
    $(BULLET_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src \
    -I$(top_srcdir)/src/tinyxml \
    -I$(top_srcdir)/src/osgPPU


#AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I/opt/local/Library/Frameworks/Python.framework/Versions/2.6/include/python2.6 -isysroot / -fno-strict-aliasing -fno-common -dynamic -DNDEBUG -fwrapv -O3 -Wall

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libspin-@SPIN_API_VERSION@.la

if HAVE_PYTHON
python_LTLIBRARIES = libSPINPyWrap.la
endif

libspin_@SPIN_API_VERSION@_la_LIBADD = \
    $(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la \
    $(top_builddir)/src/osgPPU/libosgPPU.la \
    $(top_builddir)/src/tinyxml/libtinyxml.la

libspin_@SPIN_API_VERSION@_la_LDFLAGS = \
    -export-dynamic \
    -version-info $(SPIN_LT_VERSION) \
    $(BOOST_SYSTEM_LIBS) \
    $(BOOST_FILESYSTEM_LIBS) \
    $(BOOST_PROGRAM_OPTIONS_LIBS) \
    $(BOOST_PYTHON_LIBS) \
    $(BOOST_LDFLAGS) \
    $(CPPINTROSPECTION_LIBS) \
    $(PYTHON_LDFLAGS) \
    $(LIBLO_LIBS) \
    $(OPENTHREADS_LIBS) \
    $(OSG_LIBS) \
    $(SHAREDVIDEO_LIBS) \
    $(SPATOSC_LIBS) \
    $(SPNAV_LIBS) \
    $(POCO_LIBS)
    
#	$(BOOST_THREAD_LIBS) 

# please keep those in alphabetical order:
libspin_@SPIN_API_VERSION@_la_SOURCES = \
    AnimationNode.cpp \
    AttractorNode.cpp \
    ConstraintsNode.cpp \
    CompositeViewer.cpp \
    Contour.cpp \
    DraggerTrackball.cpp \
    DraggerWith3Axes.cpp \
    DSPNode.cpp \
    EventHandler.cpp \
    Fog.cpp \
    GeometryNode.cpp \
    GridNode.cpp \
    GroupNode.cpp \
    ImageTexture.cpp \
    introspect_helpers.cpp \
    libloUtil.cpp \
    LightSource.cpp \
    Listener.cpp \
    MeasurementNode.cpp \
    MediaManager.cpp \
    Menu3D.cpp \
    ModelNode.cpp \
    nodeVisitors.cpp \
    osgUtil.cpp \
    ParticleSystem.cpp \
    pocoUtil.cpp \
    PointerNode.cpp \
    RayNode.cpp \
    ReferencedNode.cpp \
    ReferencedStateSet.cpp \
    ReporterNode.cpp \
    SceneManager.cpp \
    Shader.cpp \
    ShaderUtil.cpp \
    ShapeNode.cpp \
    SharedVideoTexture.cpp \
    SoundNode.cpp \
    SoundSpace.cpp \
    spinApp.cpp \
    spinBaseContext.cpp \
    spinClientContext.cpp \
    spinServerContext.cpp \
    spinUtil.cpp \
    SwitchNode.cpp \
    TextNode.cpp \
    UserNode.cpp \
    VideoTexture.cpp \
    ViewerManipulator.cpp

libspin_@SPIN_API_VERSION@_la_includedir = $(includedir)/spin-$(SPIN_API_VERSION)/spin
libspin_@SPIN_API_VERSION@_la_include_HEADERS = \
    $(top_srcdir)/include/AnimationNode.h \
    $(top_srcdir)/include/AttractorNode.h \
    $(top_srcdir)/include/config.h \
    $(top_srcdir)/include/CompositeViewer.h \
    $(top_srcdir)/include/ConstraintsNode.h \
    $(top_srcdir)/include/Contour.h \
    $(top_srcdir)/include/DraggerTrackball.h \
    $(top_srcdir)/include/DraggerWith3Axes.h \
    $(top_srcdir)/include/DSPNode.h \
    $(top_srcdir)/include/EventHandler.h \
    $(top_srcdir)/include/EventTypes.h \
    $(top_srcdir)/include/Fog.h \
    $(top_srcdir)/include/GeometryNode.h \
    $(top_srcdir)/include/GridNode.h \
    $(top_srcdir)/include/GroupNode.h \
    $(top_srcdir)/include/ImageTexture.h \
    $(top_srcdir)/include/introspect_helpers.h \
    $(top_srcdir)/include/libloUtil.h \
    $(top_srcdir)/include/LightSource.h \
    $(top_srcdir)/include/Listener.h \
    $(top_srcdir)/include/mainpage.h \
    $(top_srcdir)/include/MeasurementNode.h \
    $(top_srcdir)/include/MediaManager.h \
    $(top_srcdir)/include/Menu3D.h \
    $(top_srcdir)/include/ModelNode.h \
    $(top_srcdir)/include/nodeVisitors.h \
    $(top_srcdir)/include/oscParser.h \
    $(top_srcdir)/include/osgUtil.h \
    $(top_srcdir)/include/ParticleSystem.h \
    $(top_srcdir)/include/ParticleSystemUtil.h \
    $(top_srcdir)/include/pocoUtil.h \
    $(top_srcdir)/include/PointerNode.h \
    $(top_srcdir)/include/RayNode.h \
    $(top_srcdir)/include/ReferencedNode.h \
    $(top_srcdir)/include/ReferencedStateSet.h \
    $(top_srcdir)/include/ReporterNode.h \
    $(top_srcdir)/include/SceneManager.h \
    $(top_srcdir)/include/Shader.h \
    $(top_srcdir)/include/ShaderUtil.h \
    $(top_srcdir)/include/ShapeNode.h \
    $(top_srcdir)/include/SharedVideoTexture.h \
    $(top_srcdir)/include/SoundNode.h \
    $(top_srcdir)/include/SoundSpace.h \
    $(top_srcdir)/include/spinApp.h \
    $(top_srcdir)/include/spinBaseContext.h \
    $(top_srcdir)/include/spinClientContext.h \
    $(top_srcdir)/include/spinDefaults.h \
    $(top_srcdir)/include/spinLog.h \
    $(top_srcdir)/include/spinLogWX.h \
    $(top_srcdir)/include/spinServerContext.h \
    $(top_srcdir)/include/spinUtil.h \
    $(top_srcdir)/include/spinWX.h \
    $(top_srcdir)/include/SwitchNode.h \
    $(top_srcdir)/include/TextNode.h \
    $(top_srcdir)/include/UserNode.h \
    $(top_srcdir)/include/VideoTexture.h \
    $(top_srcdir)/include/ViewerManipulator.h \
    $(top_srcdir)/src/tinyxml/tinystr.h \
    $(top_srcdir)/src/tinyxml/tinyxml.h

# $(top_srcdir)/include/videoSize.h

if HAVE_PYTHON
libSPINPyWrap_la_SOURCES = python_wrappers.cpp
#libSPINPyWrap_la_LDFLAGS = -shrext ".so" # python expects .so on OSX
libSPINPyWrap_la_LIBADD = libspin-@SPIN_API_VERSION@.la \
	$(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la

libSPINPyWrapdir = `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
# `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0,"${exec_prefix}")'`

# On OSX, we need the python module to have a .so extension, but the -shrext option
# doesn't work with glibtool, so we make a symbolic link in an install hook instead.
# On Linux, this will fail because the .dylip doesn't exist. Is there any way we can
# do this only for OSX?
if HAVE_OSX
install-exec-hook:
	cd $(libSPINPyWrapdir) && $(LN_S) -f libSPINPyWrap.dylib libSPINPyWrap.so
endif

# #$(LN_S) prog-$(VERSION)$(EXEEXT) prog$(EXEEXT)
endif


if BULLET
AM_CPPFLAGS += \
    $(BULLET_CFLAGS) 
libspin_@SPIN_API_VERSION@_la_SOURCES += \
    CollisionShape.cpp\
    bulletUtil.cpp
libspin_@SPIN_API_VERSION@_la_LDFLAGS += \
    $(BULLET_LIBS)
libspin_@SPIN_API_VERSION@_la_include_HEADERS += \
    $(top_srcdir)/include/bulletUtil.h \
    $(top_srcdir)/include/CollisionShape.h
endif

if PCL
AM_CPPFLAGS += $(PCL_CFLAGS)  
libspin_@SPIN_API_VERSION@_la_SOURCES += PointCloud.cpp
libspin_@SPIN_API_VERSION@_la_LDFLAGS += $(PCL_LIBS)
libspin_@SPIN_API_VERSION@_la_include_HEADERS += $(top_srcdir)/include/PointCloud.h
endif


bin_PROGRAMS = spinserver spinviewer spindocgenerator

spinserver_SOURCES = spinServer.cpp
spinserver_LDADD = libspin-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(POCO_LIBS) $(SPNAV_LIBS)

spinviewer_SOURCES = spinViewer.cpp
spinviewer_LDADD = libspin-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(SPNAV_LIBS)

spindocgenerator_SOURCES = docGenerator.cpp
spindocgenerator_LDADD = libspin-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(POCO_LIBS) $(SPNAV_LIBS)

###############################################################################
# CUSTOM targets:

doc: spindocgenerator
	./spindocgenerator
	-cp *.html ../../html
