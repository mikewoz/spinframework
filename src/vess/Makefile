# the following file must be included:
include ../../Makefile.include

all: $(AS_OS)

# choose names of binaries:
Linux: vessServer asViewer vessServerWX
Darwin: vessServer asViewer vessServerWX vess.app

# if you have a custom verion of wxWidgets:
#WXCONFIG = $(HOME)/svn/wxMac-2.8.9/wx-config
#WXCONFIG = /usr/local/bin/wx-config

########################################################
## YOU SHOULDN'T NEED TO CHANGE ANYTHING BEYONG HERE! ##
########################################################


vpath %.cpp .  ../tinyxml

OBJS =  vessThreads.o pdUtil.o osgUtil.o asUtil.o libloUtil.o \
 	findNodeVisitor.o DebugVisitor.o \
	tinystr.o tinyxml.o tinyxmlerror.o tinyxmlparser.o \
	asSceneManager.o asMediaManager.o asReferenced.o \
	asBasicNode.o asShape.o asModel.o asVideo.o asLightSource.o asRay.o asContour.o \
	asConstraints.o asRayIntersector.o asPointer.o asMeasurementNode.o asToggle.o \
	asDSPnode.o asSoundNode.o asSoundSpace.o asSoundConnection.o \
	asCameraView.o asCameraManager.o asWindow.o
	
	#ss_rolloff_table.o ss_rolloff_manager.o \

WX_OBJS = vessWX.o wxVessMain.o wxVessEditor.o wxVessRenderer.o wxVessConfig.o \
	  wxVessPropGrid.o wxVessTreeVisitor.o wxVessTreeCtrl.o

SIMPLEVIEWER_OBJ = viewer.o asCameraManager.o asCameraView.o asWindow.o asUtil.o \
	tinystr.o tinyxml.o tinyxmlerror.o tinyxmlparser.o \



AS_CFLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG
#AS_CFLAGS += -DOSCDEBUG

# WX VARS:
ifndef WXCONFIG
  WXCONFIG = "wx-config"
endif
WX_LIBS = $(shell $(WXCONFIG) --libs --gl-libs)
ifeq ($(AS_OS),Darwin)
  WX_LIBS += -lwxcode_macu_propgrid-2.8 
else
  WX_LIBS += -lwxcode_gtk2u_propgrid-2.8
endif
WX_CXXFLAGS = $(shell $(WXCONFIG) --cxxflags)
WX_VERSION = $(shell $(WXCONFIG) --version)



ifdef OSG22
	AS_CFLAGS += -DOSG22
endif

AS_INCLUDE += -I../tinyxml -I..


#AS_LIBS += -lAudioscape

###################################################################
# GENERIC COMPILATION RULE

%.o: %.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(OSG_INCLUDE) $(WX_CXXFLAGS) $(AS_CFLAGS) $(AS_INCLUDE) -c $< -o $@

###################################################################
# TARGETS:

ALLTARGETS = vess_old asViewer viewer vessServer vessServerWX

vess_old: $(OBJS) vess.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(AS_LIBS) -lAudioscape $(OSG_LIBS)

asViewer: $(OBJS) asViewer.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(AS_LIBS) -lAudioscape $(OSG_LIBS)

viewer: $(SIMPLEVIEWER_OBJ)
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $(SIMPLEVIEWER_OBJ) $(AS_LIBS) $(OSG_LIBS)

vessServer: $(OBJS) vessServer.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(AS_LIBS) -lAudioscape $(OSG_LIBS)

vessServerWX: $(OBJS) $(WX_OBJS)
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ -lAudioscape $(WX_LIBS) $(OSG_LIBS) $(AS_LIBS)

vess.app: vessServerWX
	@echo "\n--------------- Building $@: ---------------" 
	-mkdir $@    
	-mkdir $@/Contents
	-mkdir $@/Contents/MacOS
	-mkdir $@/Contents/Resources
	-mkdir $@/Contents/Resources/English.lproj
	sed -e "s/IDENTIFIER/`echo ./ | sed -e 's,\.\./,,g' | sed -e 's,/,.,g'`/" \
	-e "s/EXECUTABLE/$</" \
	-e "s/VERSION/$(WX_VERSION)/" \
	Info.plist.in >$@/Contents/Info.plist
	echo -n 'APPL????' > $@/Contents/PkgInfo
	cp vessServerWX $@/Contents/MacOS
	#cp ../../lib/Darwin_i386/libAudioscape.dylib $@/Contents/Resources
	cp vess.icns $@/Contents/Resources/vessServerWX.icns


########################################################
install:
	@echo "make install NOT YET DONE. Where do we install to??"


########################################################
clean:
	-rm -f $(ALLTARGETS) *.o 
