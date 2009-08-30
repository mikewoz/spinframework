# the following file must be included:
include ../../Makefile.include

# choose names of library:
LIBNAME = libSPIN

all: $(OS)

WRAPPER_OBJ = asBasicNode.o asReferenced.o asShape.o asModel.o asVideo.o asLightSource.o asRay.o asContour.o asConstraints.o asPointer.o asMeasurementNode.o asSoundNode.o asSoundSpace.o asDSPnode.o asSoundConnection.o userNode.o sharedVideoNode.o


#SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG -Wall
SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG


###################################################################
# GENERIC COMPILATION RULE:

%.o: %.cpp
	$(CXX) $(SPIN_FLAGS) $(SPIN_INCLUDE) $(OSG_INCLUDE) $(BOOST_INCLUDE) -c $< -o $@

###################################################################
# TARGETS:


Linux: $(WRAPPER_OBJ)
	$(CXX) --export-dynamic -shared -Wl,-soname,$(LIBNAME).so -o $(LIBNAME).so $(WRAPPER_OBJ) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)

Darwin: $(WRAPPER_OBJ)
	$(CXX) -dynamiclib -undefined suppress -flat_namespace -o $(LIBNAME).dylib $(WRAPPER_OBJ) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)


########################################################
install:
	@echo "INSTALLING ${LIBNAME} to ${SPIN_INSTALL_PREFIX}";
	-cp $(LIBNAME).* ${SPIN_INSTALL_PREFIX}/lib

########################################################
clean:
	-rm -f $(LIBNAME).* *.o 
