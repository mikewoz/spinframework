AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I$(top_srcdir)/include $(PYTHON_CPPFLAGS) $(OSG_CFLAGS) $(OSGINTROSPECTION_CFLAGS)
#AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/src/tinyxml -I/opt/local/Library/Frameworks/Python.framework/Versions/2.6/include/python2.6 -isysroot / -fno-strict-aliasing -fno-common -dynamic -DNDEBUG -fwrapv -O3 -Wall

# use lib_LTLIBRARIES to build a shared lib:
#lib_LTLIBRARIES = libSPIN.la libSPINPyWrap.la
lib_LTLIBRARIES = libSPIN.la
python_LTLIBRARIES = libSPINPyWrap.la

libSPIN_la_LIBADD = \
	$(top_builddir)/src/osgWrappers/introspection/libSPINwrappers.la \
	$(top_builddir)/src/tinyxml/libtinyxml.la

libSPIN_la_LDFLAGS = \
	$(PYTHON_LDFLAGS) \
	$(BOOST_LDFLAGS) \
	$(BOOST_FILESYSTEM_LIB) \
	-l$(BOOST_PYTHON_LIB) \
    $(OPENTHREADS_LIBS) \
    $(OSG_LIBS) \
	$(OSGINTROSPECTION_LIB) $(LIBLO_LDFLAGS) $(BOOST_THREAD_LIB) $(SHARED_VIDEO_LIB)

# please keep those in alphabetical order:
libSPIN_la_SOURCES = \
	AttractorNode.cpp \
	ConstraintsNode.cpp \
	DSPNode.cpp \
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

libSPIN_la_includedir = $(includedir)/spinFramework
libSPIN_la_include_HEADERS = $(top_srcdir)/include/*.h $(top_srcdir)/src/tinyxml/*.h

libSPINPyWrap_la_SOURCES = python_wrappers.cpp
libSPINPyWrap_la_LDFLAGS = -shrext ".so" # python expects .so on OSX
libSPINPyWrap_la_LIBADD = libSPIN.la

libSPINPyWrapdir = `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0)'`
# `python -c 'from distutils import sysconfig; print sysconfig.get_python_lib(1,0,"${exec_prefix}")'`


bin_PROGRAMS = spinserver spinviewer spindocgenerator

spinserver_SOURCES = spinServer.cpp
spinserver_LDADD = libSPIN.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(OSGINTROSPECTION_LIBS)

spinviewer_SOURCES = spinViewer.cpp
spinviewer_LDADD = libSPIN.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(OSGINTROSPECTION_LIBS)

spindocgenerator_SOURCES = docGenerator.cpp
spindocgenerator_LDADD = libSPIN.la $(OPENTHREADS_LIBS) $(OSG_LIBS) $(OSGINTROSPECTION_LIBS)


###############################################################################
# CUSTOM targets:

#OSG_VER = $(shell osgversion --so-number)
#OPENTHREADS_VER= $(shell osgversion --openthreads-soversion-number)

#OSX_OSG_LIBS = osg osgGA osgSim osgDB osgManipulator osgText osgUtil osgFX osgParticle osgViewer osgIntrospection


# WARNING: the .app generation script pulls some libs (eg, libSPIN) from /usr/local
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
#install_name_tool -change libosgIntrospection.55.dylib @executable_path/../libs/libosgIntrospection.55.dylib $@/Contents/MacOS/spinviewer


#install_name_tool -change libSPIN.dylib @executable_path/../libs/libSPIN.dylib $@/Contents/MacOS/spinviewer
#-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
#-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
#-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents

doc:
	./spindocgenerator
	-cp *.html ../../doxygen/html

