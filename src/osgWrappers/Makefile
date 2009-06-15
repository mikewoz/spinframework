# the following file must be included:
include ../../Makefile.include

# choose names of library:
LIBNAME = libSPIN

all: $(AS_OS)

WRAPPER_OBJ = asBasicNode.o asReferenced.o asShape.o asModel.o asVideo.o asLightSource.o asRay.o asContour.o asConstraints.o asPointer.o asMeasurementNode.o asSoundNode.o asSoundSpace.o asDSPnode.o asSoundConnection.o
#WRAPPER_OBJ = asBasicNode.o asReferenced.o asShape.o asModel.o asTexture.o asLightSource.o asRay.o asContour.o asConstraints.o asRayIntersector.o asPointer.o asMeasurementNode.o asToggle.o asSoundNode.o asSoundSpace.o asDSPnode.o asSoundConnection.o


AS_CFLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG -Wall


###################################################################
# GENERIC COMPILATION RULE:

%.o: %.cpp
	$(CXX) $(AS_FLAGS) $(AS_INCLUDE) $(OSG_INCLUDE) -c $< -o $@

###################################################################
# TARGETS:


Linux: $(WRAPPER_OBJ)
	$(CXX) --export-dynamic -shared -Wl,-soname,$(LIBNAME).so -o $(LIBNAME).so $(WRAPPER_OBJ) $(AS_LIBS) $(OSG_LIBS)
	-cp $(LIBNAME).* $(AS_LIBDIR)

Darwin: $(WRAPPER_OBJ)
	$(CXX) -dynamiclib -undefined suppress -flat_namespace -o $(LIBNAME).dylib $(WRAPPER_OBJ) $(AS_LIBS) $(OSG_LIBS)
	-cp $(LIBNAME).* $(AS_LIBDIR)


########################################################
install:
	-cp $(LIBNAME).* $(AS_LIBDIR)

########################################################
clean:
	-rm -f $(LIBNAME).* *.o 
