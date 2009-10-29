# the following file must be included:
include ../../Makefile.include

all: $(OS)
#	@echo "OS=${OS}";
#	@echo "curdir=${CURDIR}";
#	@echo "pwd="${PWD};
#	@echo "SPIN_PATH=${SPIN_PATH}";

PROGRAMS = spinServer spinViewer spin

# choose targets:
Linux: libSPIN $(PROGRAMS)
Darwin: libSPIN $(PROGRAMS)

apps: spin.app spinViewer.app

# if you have a custom verion of wxWidgets:
#WXCONFIG = $(HOME)/svn/wxMac-2.8.9/wx-config
#WXCONFIG = /usr/local/bin/wx-config
#WXCONFIG = /usr/bin/wx-config

########################################################
## YOU SHOULDN'T NEED TO CHANGE ANYTHING BEYONG HERE! ##
########################################################


vpath %.cpp .  ../tinyxml $(APPSINK_PATH) ../tests/bullet_physics


OBJS = spinContext.o osgUtil.o spinUtil.o libloUtil.o \
       tinystr.o tinyxml.o tinyxmlerror.o tinyxmlparser.o \
       SceneManager.o MediaManager.o ReferencedNode.o \
       GroupNode.o ShapeNode.o ModelNode.o VideoNode.o LightSource.o \
       RayNode.o Contour.o ConstraintsNode.o PointerNode.o \
       TextNode.o MeasurementNode.o UserNode.o \
       DSPNode.o Listener.o SoundNode.o SoundSpace.o SoundConnection.o \
       SharedVideoNode.o sharedVideoBuffer.o 

WX_OBJS = spinWX.o wxSpinMain.o wxSpinEditor.o wxSpinRenderer.o \
	  wxSpinPropGrid.o wxSpinSettings.o wxSpinTreeVisitor.o \
	  wxSpinTreeCtrl.o


SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus

ifeq ($(OS),Linux)
	SPIN_FLAGS += -D_SPINDEBUG
endif

SPIN_FLAGS += -D_SPINDEBUG

#SPIN_FLAGS += -DOSCDEBUG

DEV_LDFLAGS = -L. -L../osgWrappers -Wl,-rpath,. -Wl,-rpath,../osgWrappers -Wl,-rpath,$(SPIN_INSTALL_PREFIX)/lib

SPIN_LIBS += -llo -lSPIN

ifeq ($(OS),Darwin)
  BOOST_LIBS += -lboost_thread-mt -lboost_filesystem-mt
else
  BOOST_LIBS += -lboost_thread -lboost_filesystem
endif

###################################################################
# WX VARS:
ifndef WXCONFIG
  WXCONFIG = "wx-config"
endif
WX_LIBTYPE = $(patsubst wx_%,%,$(shell $(WXCONFIG) --basename))

WX_LIBS = $(shell $(WXCONFIG) --libs --gl-libs)
WX_CXXFLAGS = $(shell $(WXCONFIG) --cxxflags)
WX_VERSION = $(shell $(WXCONFIG) --version)

# wxPropGrid library:
WX_LIBS += -lwxcode_$(WX_LIBTYPE)_propgrid-2.8

###################################################################
# GENERIC COMPILATION RULE

%.o: %.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(OSG_INCLUDE) $(PYTHON_INCLUDE) $(WX_CXXFLAGS) $(SPIN_FLAGS) $(SPIN_INCLUDE) $(BOOST_INCLUDE) -c $< -o $@


###################################################################
# TARGETS:



# libSPIN is the core dynamic library that enables SPIN to be included
# in other apps:

ifeq ($(OS),Darwin)
libSPIN: libSPIN.dylib
else
libSPIN: libSPIN.so
endif

libSPIN.dylib: $(OBJS)
	@echo "\n------------- creating $@: -------------"
	$(CXX) -dynamiclib -undefined suppress -flat_namespace -o $@ $(OBJS) -lSPINwrappers -L../osgWrappers $(OSG_LIBS) $(BOOST_LIBS) $(PYTHON_LIBS)

libSPIN.so: $(OBJS)
	@echo "\n------------- creating $@: -------------" 
	$(CXX) --export-dynamic -shared -Wl,-soname,$@ -o $@ $(OBJS) -lSPINwrappers -L../osgWrappers $(OSG_LIBS) $(BOOST_LIBS) $(PYTHONS_LIBS)

# spinPython is a library created in Boost.Python that exposes core
# elements for use in Python scripts:

python: python_$(OS)

python_wrappers.o: python_wrappers.cpp
	@echo "--------------- compiling $@  ---------------" 
	$(CXX) $(SPIN_INCLUDE) $(PYTHON_INCLUDE) $(BOOST_INCLUDE) -c $< -o $@

python_Darwin: python_wrappers.o 
	@echo "\n------------- Creating spinFramework Python module: -------------"
	$(CXX) -shared -module -undefined suppress -flat_namespace -o spinFramework.so $(OBJS) python_wrappers.o -lSPINwrappers -L../osgWrappers -llo -lboost_python-mt $(PYTHON_LIBS) $(OSG_LIBS) $(BOOST_LIBS) $(SPIN_LIBS)

python_Linux: python_wrappers.o 
	@echo "\n------------- Creating spinFramework Python module: -------------"
	$(CXX) -shared -module -o spinFramework.so python_wrappers.o -lboost_python-mt $(DEV_LDFLAGS) $(PYTHON_LIBS) $(OSG_LIBS) $(BOOST_LIBS) $(SPIN_LIBS)


# docGenerator is an app that uses osgIntrospection and doxygen to
# create customized docs for SPIN:
doc: docGenerator
docGenerator: docGenerator.o
	@echo "\n------------- Generating OSC Documentation: -------------" 
	$(CXX) -o docGenerator $^ $(DEV_LD_FLAGS) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)
	./docGenerator
	-cp *.html ../../doxygen/html

# spinViewer is a simple OSG viewer client:
spinViewer: spinViewer.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(DEV_LDFLAGS) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)

# spinServer is a commandline SPIN server instance:
spinServer: spinServer.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(DEV_LDFLAGS) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)

# spin is a GUI (using WX Widgets) encompassing most features of SPIN:
spin: $(WX_OBJS)
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(DEV_LDFLAGS) $(SPIN_LIBS) $(WX_LIBS) $(OSG_LIBS) $(BOOST_LIBS)

bullet-test: bullet-test.o
	$(CXX) -o $@ $^ $(OSG_LIBS) -lbulletdynamics -lbulletcollision -lLinearMath -L/usr/local/include/BulletCollision 

# spin.app creates an OSX application bundle for spin:
spin.app: spin
	@echo "\n--------------- Building $@: ---------------" 
	-mkdir $@    
	-mkdir $@/Contents
	-mkdir $@/Contents/Frameworks
	-mkdir $@/Contents/libs
	-mkdir $@/Contents/MacOS
	-mkdir $@/Contents/Resources
	-mkdir $@/Contents/Resources/English.lproj
	sed -e "s/IDENTIFIER/`echo ./ | sed -e 's,\.\./,,g' | sed -e 's,/,.,g'`/" \
	-e "s/EXECUTABLE/$</" \
	-e "s/VERSION/$(WX_VERSION)/" \
	spin.plist.in >$@/Contents/Info.plist
	echo -n 'APPL????' > $@/Contents/PkgInfo
	cp spin $@/Contents/MacOS
	install_name_tool -change libSPIN.dylib @executable_pathlibSPIN.dylib $@/Contents/MacOS/spin	
	cp libSPIN.dylib $@/Contents/libs
	-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
	-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
	-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents
	-cp -R ../Resources/images $@/Contents/Resources
	cp ../Resources/images/logo_spin.icns $@/Contents/Resources
	./dylibbundler -of -b -x $@/Contents/MacOS/spin -d $@/Contents/libs

# spinViewer.app creates an OSX application bundle for spin:
spinViewer.app: spinViewer
	@echo "\n--------------- Building $@: ---------------" 
	-mkdir $@    
	-mkdir $@/Contents
	-mkdir $@/Contents/Frameworks
	-mkdir $@/Contents/libs
	-mkdir $@/Contents/MacOS
	-mkdir $@/Contents/Resources
	-mkdir $@/Contents/Resources/English.lproj
	sed -e "s/IDENTIFIER/`echo ./ | sed -e 's,\.\./,,g' | sed -e 's,/,.,g'`/" \
	-e "s/EXECUTABLE/$</" \
	-e "s/VERSION/$(WX_VERSION)/" \
	spinViewer.plist.in >$@/Contents/Info.plist
	echo -n 'APPL????' > $@/Contents/PkgInfo
	cp spinViewer $@/Contents/MacOS
	install_name_tool -change libSPIN.dylib @executable_path/libSPIN.dylib $@/Contents/MacOS/spinViewer
	cp libSPIN.dylib $@/Contents/libs
	-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
	-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
	-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents
	cp ../Resources/images/logo_spinViewer.icns $@/Contents/Resources
	./dylibbundler -of -b -x $@/Contents/MacOS/spinViewer -d $@/Contents/libs



########################################################
install:
	@echo "\nINSTALLING to ${SPIN_INSTALL_PREFIX}";
	-cp libSPIN.* ${SPIN_INSTALL_PREFIX}/lib
	-cp ${PROGRAMS} ${SPIN_INSTALL_PREFIX}/bin
	#mkdir -p ${SPIN_INSTALL_PREFIX}/share/spinFramework
	-cp -R ../Resources ${SPIN_INSTALL_PREFIX}/share/spinFramework


########################################################
cleanb:
	-rm -rf $(PROGRAMS) libSPIN.*	

clean:
	-rm -rf $(PROGRAMS) libSPIN.* *.o	
