AM_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(CPPINTROSPECTION_CFLAGS) \
    $(PYTHON_CPPFLAGS) \
    $(LIBLO_CFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) \
    $(SHAREDVIDEO_CFLAGS) \
    $(SPATOSC_CFLAGS) \
    -I$(top_srcdir)/include \
    -I$(top_srcdir)/src/tinyxml

#AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I/opt/local/Library/Frameworks/Python.framework/Versions/2.6/include/python2.6 -isysroot / -fno-strict-aliasing -fno-common -dynamic -DNDEBUG -fwrapv -O3 -Wall

# use lib_LTLIBRARIES to build a shared lib:
lib_LTLIBRARIES = libspin_@SPIN_API_VERSION@.la
python_LTLIBRARIES = libSPINPyWrap.la

libspin_@SPIN_API_VERSION@_la_LIBADD = \
    $(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la \
    $(top_builddir)/src/tinyxml/libtinyxml.la

libspin_@SPIN_API_VERSION@_la_LDFLAGS = \
    -export-dynamic \
    -version-info $(SPIN_LT_VERSION) \
    $(BOOST_FILESYSTEM_LIB) \
    $(BOOST_LDFLAGS) \
    $(CPPINTROSPECTION_LIBS) \
    $(PYTHON_LDFLAGS) \
    -l$(BOOST_PYTHON_LIB) \
    $(BOOST_THREAD_LIB) \
    $(LIBLO_LIBS) \
    $(OPENTHREADS_LIBS) \
    $(OSG_LIBS) \
    $(SHAREDVIDEO_LIBS) \
    $(SPATOSC_LIBS)

# please keep those in alphabetical order:
libspin_@SPIN_API_VERSION@_la_SOURCES = \
    AttractorNode.cpp \
    ConstraintsNode.cpp \
    DSPNode.cpp \
    EventHandler.cpp \
    Fog.cpp \
    GridNode.cpp \
    ReferencedNode.cpp \
    SceneManager.cpp \
    UserNode.cpp \
    osgUtil.cpp \
    spinApp.cpp \
    AnimationNode.cpp \
    Contour.cpp \
    GroupNode.cpp \
    ImageTexture.cpp \
    LightSource.cpp \
    Listener.cpp \
    MeasurementNode.cpp \
    MediaManager.cpp \
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
    VideoTexture.cpp \
    ViewerManipulator.cpp \
    libloUtil.cpp \
    nodeVisitors.cpp \
    spinBaseContext.cpp \
    spinClientContext.cpp \
    spinServerContext.cpp \
    spinUtil.cpp \
    introspect_helpers.cpp

libspin_@SPIN_API_VERSION@_la_includedir = $(includedir)/spinFramework
libspin_@SPIN_API_VERSION@_la_include_HEADERS = \
    $(top_srcdir)/include/AnimationNode.h \
    $(top_srcdir)/include/AttractorNode.h \
    $(top_srcdir)/include/config.h \
    $(top_srcdir)/include/ConstraintsNode.h \
    $(top_srcdir)/include/Contour.h \
    $(top_srcdir)/include/DSPNode.h \
    $(top_srcdir)/include/EventHandler.h \
    $(top_srcdir)/include/EventTypes.h \
    $(top_srcdir)/include/Fog.h \
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
    $(top_srcdir)/include/PointerNode.h \
    $(top_srcdir)/include/RayNode.h \
    $(top_srcdir)/include/ReferencedNode.h \
    $(top_srcdir)/include/ReferencedStateSet.h \
    $(top_srcdir)/include/ReporterNode.h \
    $(top_srcdir)/include/SceneManager.h \
    $(top_srcdir)/include/ShapeNode.h \
    $(top_srcdir)/include/SharedVideoTexture.h \
    $(top_srcdir)/include/SoundConnection.h \
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

libSPINPyWrap_la_SOURCES = python_wrappers.cpp
#libSPINPyWrap_la_LDFLAGS = -shrext ".so" # python expects .so on OSX
libSPINPyWrap_la_LIBADD = libspin_@SPIN_API_VERSION@.la \
	$(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la

libSPINPyWrapdir = `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
# `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0,"${exec_prefix}")'`

# On OSX, we need the python module to have a .so extension, but the -shrext option
# doesn't work with glibtool, so we make a symbolic link in an install hook instead.
# On Linux, this will fail because the .dylip doesn't exist. Is there any way we can
# do this only for OSX?
install-exec-hook:
	cd $(libSPINPyWrapdir) && $(LN_S) -f libSPINPyWrap.dylib libSPINPyWrap.so

#$(LN_S) prog-$(VERSION)$(EXEEXT) prog$(EXEEXT)


bin_PROGRAMS = spinserver spinviewer spindocgenerator

spinserver_SOURCES = spinServer.cpp
spinserver_LDADD = libspin_@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS)

spinviewer_SOURCES = spinViewer.cpp
spinviewer_LDADD = libspin_@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS)

spindocgenerator_SOURCES = docGenerator.cpp
spindocgenerator_LDADD = libspin_@SPIN_API_VERSION@.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(CPPINTROSPECTION_LIBS) $(PYTHON_LDFLAGS)

###############################################################################
# CUSTOM targets:

#OSG_VER = $(shell osgversion --so-number)
#OPENTHREADS_VER= $(shell osgversion --openthreads-soversion-number)

#OSX_OSG_LIBS = osg osgGA osgSim osgDB osgManipulator osgText osgUtil osgFX osgParticle osgViewer cppintrospection

# WARNING: the .app generation script pulls some libs (eg, libspin_@SPIN_API_VERSION@) from /usr/local
# so you must do a 'make install' before this will work (TODO: fix this)

spinviewer.app: spinviewer$(EXEEXT)
	@echo "\n--------------- Building $@: ---------------" 
	-mkdir $@    
	-mkdir $@/Contents
	-mkdir $@/Contents/Frameworks
	-mkdir $@/Contents/libs
	-mkdir $@/Contents/MacOS
	-mkdir $@/Contents/Resources
	-mkdir $@/Contents/Resources/fonts
	-mkdir $@/Contents/Resources/images
	-mkdir $@/Contents/Resources/scripts
	-mkdir $@/Contents/Resources/English.lproj
	sed -e "s/IDENTIFIER/`echo ./ | sed -e 's,\.\./,,g' | sed -e 's,/,.,g'`/" \
	-e "s/EXECUTABLE/$</" \
	-e "s/VERSION/$(VERSION)/" \
	spinviewer.plist.in >$@/Contents/Info.plist
	echo -n 'APPL????' > $@/Contents/PkgInfo
	cp /usr/local/bin/spinviewer $@/Contents/MacOS
	../../util/dylibbundler/dylibbundler -of -b -x $@/Contents/MacOS/spinviewer -d $@/Contents/libs

#	cp ../Resources/fonts/*.ttf $@/Contents/Resources/fonts
#	cp ../Resources/images/*.icns ../Resources/images/*.xpm ../Resources/images/*.gif ../Resources/images/*.png $@/Contents/Resources/images
#	cp ../Resources/scripts/*.py $@/Contents/Resources/scripts

#cp .libs/spinviewer $@/Contents/MacOS

#	install_name_tool -change libOpenThreads.$(OPENTHREADS_VER).dylib /usr/local/lib/libOpenThreads.$(OPENTHREADS_VER).dylib $@/Contents/MacOS/spinviewer
#-cp -R /usr/local/lib/osgPlugins-* $@/Contents/libs
#-cp /usr/local/lib/libosg*.dylib $@/Contents/libs
#-cp /usr/local/lib/libOpenThreads.*.dylib $@/Contents/libs

#install_name_tool -change libosg.55.dylib @executable_path/../libs/libosg.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgGA.55.dylib @executable_path/../libs/libosgGA.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgSim.55.dylib @executable_path/../libs/libosgSim.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgDB.55.dylib @executable_path/../libs/libosgDB.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgManipulator.55.dylib @executable_path/../libs/libosgManipulator.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgText.55.dylib @executable_path/../libs/libosgText.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgUtil.55.dylib @executable_path/../libs/libosgUtil.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgFX.55.dylib @executable_path/../libs/libosgFX.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgParticle.55.dylib @executable_path/../libs/libosgParticle.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libosgViewer.55.dylib @executable_path/../libs/libosgViewer.55.dylib $@/Contents/MacOS/spinviewer
#install_name_tool -change libcppintrospection.55.dylib @executable_path/../libs/libcppintrospection.55.dylib $@/Contents/MacOS/spinviewer

#install_name_tool -change libSPIN.dylib @executable_path/../libs/libSPIN.dylib $@/Contents/MacOS/spinviewer
#-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
#-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
#-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents

doc:
	./spindocgenerator
	-cp *.html ../../doxygen/html
