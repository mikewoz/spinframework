AM_CPPFLAGS = \
    -std=gnu++0x \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(LIBLO_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) \
    $(SHAREDVIDEO_CFLAGS) \
    $(SPATOSC_CFLAGS) \
    $(BULLET_CFLAGS) \
    $(TINYXML_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src \
    -I$(top_srcdir)/src/osgPPU

#AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I/opt/local/Library/Frameworks/Python.framework/Versions/2.6/include/python2.6 -isysroot / -fno-strict-aliasing -fno-common -dynamic -DNDEBUG -fwrapv -O3 -Wall

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libspinframework-@SPIN_API_VERSION@.la

if HAVE_PYTHON
python_LTLIBRARIES = spinframework.la
endif

libspinframework_@SPIN_API_VERSION@_la_LIBADD = \
    $(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la \
    $(top_builddir)/src/osgPPU/libosgPPU.la

libspinframework_@SPIN_API_VERSION@_la_LDFLAGS = \
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
    $(TINYXML_LIBS) \
    $(POCO_LIBS)

# please keep those in alphabetical order:
# Warning: bullet-specific files are not listed here!
libspinframework_@SPIN_API_VERSION@_la_SOURCES = \
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
    LODNode.cpp \
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

libspinframework_@SPIN_API_VERSION@_la_includedir = $(includedir)/spinframework-$(SPIN_API_VERSION)/spin
# Warning: bullet-specific files are not listed here!
libspinframework_@SPIN_API_VERSION@_la_include_HEADERS = \
    $(top_srcdir)/include/animationnode.h \
    $(top_srcdir)/include/attractornode.h \
    $(top_srcdir)/include/compositeviewer.h \
    $(top_srcdir)/include/constraintsnode.h \
    $(top_srcdir)/include/contour.h \
    $(top_srcdir)/include/dspnode.h \
    $(top_srcdir)/include/draggertrackball.h \
    $(top_srcdir)/include/draggerwith3axes.h \
    $(top_srcdir)/include/eventhandler.h \
    $(top_srcdir)/include/eventtypes.h \
    $(top_srcdir)/include/fog.h \
    $(top_srcdir)/include/geometrynode.h \
    $(top_srcdir)/include/gridnode.h \
    $(top_srcdir)/include/groupnode.h \
    $(top_srcdir)/include/imagetexture.h \
    $(top_srcdir)/include/kinectcloud.h \
    $(top_srcdir)/include/lodnode.h \
    $(top_srcdir)/include/lightsource.h \
    $(top_srcdir)/include/listener.h \
    $(top_srcdir)/include/menu3d.h \
    $(top_srcdir)/include/modelnode.h \
    $(top_srcdir)/include/particlesystem.h \
    $(top_srcdir)/include/particlesystemutil.h \
    $(top_srcdir)/include/pointcloud.h \
    $(top_srcdir)/include/pointernode.h \
    $(top_srcdir)/include/raynode.h \
    $(top_srcdir)/include/referencednode.h \
    $(top_srcdir)/include/referencedstateset.h \
    $(top_srcdir)/include/reporternode.h \
    $(top_srcdir)/include/scenemanager.h \
    $(top_srcdir)/include/shader.h \
    $(top_srcdir)/include/shaderutil.h \
    $(top_srcdir)/include/shapenode.h \
    $(top_srcdir)/include/sharedvideotexture.h \
    $(top_srcdir)/include/soundnode.h \
    $(top_srcdir)/include/soundspace.h \
    $(top_srcdir)/include/switchnode.h \
    $(top_srcdir)/include/syphontexture.h \
    $(top_srcdir)/include/textnode.h \
    $(top_srcdir)/include/usernode.h \
    $(top_srcdir)/include/videotexture.h \
    $(top_srcdir)/include/viewermanipulator.h \
    $(top_srcdir)/include/config.h \
    $(top_srcdir)/include/dofppu.h \
    $(top_srcdir)/include/introspect_helpers.h \
    $(top_srcdir)/include/libloutil.h \
    $(top_srcdir)/include/mainpage.h \
    $(top_srcdir)/include/maskppu.h \
    $(top_srcdir)/include/motionblurppu.h \
    $(top_srcdir)/include/nodevisitors.h \
    $(top_srcdir)/include/oscparser.h \
    $(top_srcdir)/include/osgutil.h \
    $(top_srcdir)/include/outlineppu.h \
    $(top_srcdir)/include/pocoutil.h \
    $(top_srcdir)/include/spinapp.h \
    $(top_srcdir)/include/spinbasecontext.h \
    $(top_srcdir)/include/spinclientcontext.h \
    $(top_srcdir)/include/spindefaults.h \
    $(top_srcdir)/include/spinlog.h \
    $(top_srcdir)/include/spinlogwx.h \
    $(top_srcdir)/include/spinservercontext.h \
    $(top_srcdir)/include/spinutil.h \
    $(top_srcdir)/include/spinwx.h \
    $(top_srcdir)/include/ssaoppu.h \
    $(top_srcdir)/include/util.h \
    $(top_srcdir)/include/videosize.h

if HAVE_PYTHON
spinframework_la_LDFLAGS = -module
spinframework_la_SOURCES = python_wrappers.cpp
#spinframework_la_LDFLAGS = -shrext ".so" # python expects .so on OSX
spinframework_la_LIBADD = libspinframework-@SPIN_API_VERSION@.la \
	$(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la


spinframeworkdir = `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
# `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0,"${exec_prefix}")'`

# On OSX, we need the python module to have a .so extension, but the -shrext option
# doesn't work with glibtool, so we make a symbolic link in an install hook instead.
# On Linux, this will fail because the .dylip doesn't exist. Is there any way we can
# do this only for OSX?
if HAVE_OSX
install-exec-hook:
	cd $(spinframeworkdir) && $(LN_S) -f spinframework.dylib spinframework.so
endif

# #$(LN_S) prog-$(VERSION)$(EXEEXT) prog$(EXEEXT)
endif


if BULLET
AM_CPPFLAGS += \
    $(BULLET_CFLAGS) 
libspinframework_@SPIN_API_VERSION@_la_SOURCES += \
    CollisionShape.cpp\
    bulletUtil.cpp
libspinframework_@SPIN_API_VERSION@_la_LDFLAGS += \
    $(BULLET_LIBS)
libspinframework_@SPIN_API_VERSION@_la_include_HEADERS += \
    $(top_srcdir)/include/bulletutil.h \
    $(top_srcdir)/include/collisionshape.h
endif

if PCL
AM_CPPFLAGS += $(PCL_CFLAGS) \
    -I/usr/include/ni
libspinframework_@SPIN_API_VERSION@_la_SOURCES += PointCloud.cpp
#the following fixes the bugy pc file in libflann
#libspinframework_@SPIN_API_VERSION@_la_LDFLAGS += $(PCL_LIBS)
libspinframework_@SPIN_API_VERSION@_la_LDFLAGS += -lpcl_io -lOpenNI -lusb-1.0 -lpcl_filters -lpcl_sample_consensus -lpcl_search -lpcl_kdtree -lflann_cpp -lpcl_octree -lpcl_common
libspinframework_@SPIN_API_VERSION@_la_include_HEADERS += $(top_srcdir)/include/pointcloud.h
endif


bin_PROGRAMS = spinserver spinviewer spindocgenerator

spinserver_SOURCES = spinServer.cpp
spinserver_LDADD = libspinframework-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(POCO_LIBS) $(SPNAV_LIBS) $(TINYXML_LIBS)

spinviewer_SOURCES = spinViewer.cpp
spinviewer_LDADD = libspinframework-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(SPNAV_LIBS) $(TINYXML_LIBS)

spindocgenerator_SOURCES = docGenerator.cpp
spindocgenerator_LDADD = libspinframework-@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS) $(LIBLO_LIBS) $(BOOST_PROGRAM_OPTIONS_LIBS) $(BOOST_SYSTEM_LIBS) $(BOOST_FILESYSTEM_LIBS) $(POCO_LIBS) $(SPNAV_LIBS) $(TINYXML_LIBS)

###############################################################################
# CUSTOM targets:

doc: spindocgenerator
	./spindocgenerator
	-cp *.html ../../html
