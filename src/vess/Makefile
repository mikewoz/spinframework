# the following file must be included:
include ../../Makefile.include

all: $(OS)
#	@echo "OS=${OS}";
#	@echo "curdir=${CURDIR}";
#	@echo "pwd="${PWD};
#	@echo "SPIN_PATH=${SPIN_PATH}";

# choose names of binaries:
Linux: vess spinViewer spin
Darwin: vess spinViewer spin

apps: spin.app spinViewer.app

# if you have a custom verion of wxWidgets:
#WXCONFIG = $(HOME)/svn/wxMac-2.8.9/wx-config
#WXCONFIG = /usr/local/bin/wx-config
#WXCONFIG = /usr/bin/wx-config

########################################################
## YOU SHOULDN'T NEED TO CHANGE ANYTHING BEYONG HERE! ##
########################################################


vpath %.cpp .  ../tinyxml

OBJS =  vessThreads.o pdUtil.o osgUtil.o asUtil.o libloUtil.o \
 	findNodeVisitor.o DebugVisitor.o  \
	tinystr.o tinyxml.o tinyxmlerror.o tinyxmlparser.o \
	asSceneManager.o asMediaManager.o asReferenced.o \
	asBasicNode.o asShape.o asModel.o asVideo.o asLightSource.o asRay.o \
	asContour.o asConstraints.o asRayIntersector.o asPointer.o \
	asMeasurementNode.o asToggle.o userNode.o \
	asDSPnode.o asSoundNode.o asSoundSpace.o asSoundConnection.o \
	asCameraView.o asCameraManager.o asWindow.o

WX_OBJS = vessWX.o wxVessMain.o wxVessEditor.o wxVessRenderer.o \
	  wxVessPropGrid.o wxVessSettings.o wxVessTreeVisitor.o \
	  wxVessTreeCtrl.o

SIMPLEVIEWER_OBJ = viewer.o asCameraManager.o asCameraView.o asWindow.o asUtil.o \
	tinystr.o tinyxml.o tinyxmlerror.o tinyxmlparser.o \



SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus

ifeq ($(OS),Linux)
	SPIN_FLAGS += -D_SPINDEBUG
endif

SPIN_FLAGS += -D_SPINDEBUG

#SPIN_FLAGS += -DOSCDEBUG

SPIN_LIBS += -llo -L../osgWrappers


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
	$(CXX) $(OSG_INCLUDE) $(WX_CXXFLAGS) $(SPIN_FLAGS) $(SPIN_INCLUDE) -c $< -o $@

###################################################################
# TARGETS:

ALLTARGETS = docGenerator spinViewer vess spin

doc: docGenerator
docGenerator: $(OBJS) docGenerator.o
	@echo "\n------------- Generating OSC Documentation: -------------" 
	$(CXX) -o docGenerator $^ $(SPIN_LIBS) -lSPIN $(OSG_LIBS)
	./docGenerator
	-cp *.html ../../doxygen/html

spinViewer: $(OBJS) spinViewer.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(SPIN_LIBS) -lSPIN $(OSG_LIBS)

vess: $(OBJS) vess.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(SPIN_LIBS) -lSPIN $(OSG_LIBS)

spin: $(OBJS) $(WX_OBJS)
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ -lSPIN $(WX_LIBS) $(OSG_LIBS) $(SPIN_LIBS)

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
	install_name_tool -change libSPIN.dylib @executable_path/../libs/libSPIN.dylib $@/Contents/MacOS/spin	
	cp ../osgWrappers/libSPIN.dylib $@/Contents/libs
	-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
	-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
	-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents
	-cp -R ../Resources/images $@/Contents/Resources
	cp spin.icns $@/Contents/Resources/spin.icns
	./dylibbundler -of -b -x $@/Contents/MacOS/spin -d $@/Contents/libs

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
	install_name_tool -change libSPIN.dylib @executable_path/../libs/libSPIN.dylib $@/Contents/MacOS/spinViewer
	cp ../osgWrappers/libSPIN.dylib $@/Contents/libs
	-cp -R /Library/Frameworks/osg* $@/Contents/Frameworks
	-cp -R /Library/Frameworks/OpenThreads.framework $@/Contents/Frameworks
	-cp -R /Library/Application\ Support/OpenSceneGraph/PlugIns $@/Contents
	cp logo_spinViewer.icns $@/Contents/Resources
	./dylibbundler -of -b -x $@/Contents/MacOS/spinViewer -d $@/Contents/libs

########################################################
install:
	@echo "make install NOT YET DONE. Where do we install to??"


########################################################
clean:
	-rm -rf $(ALLTARGETS) *.o	
