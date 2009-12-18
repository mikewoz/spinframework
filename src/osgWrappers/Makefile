# the following file must be included:
include ../../Makefile.include

# choose names of library:
LIBNAME = libSPINwrappers

#all: $(OS) staticlib
all: $(OS)

NODELIST = ReferencedNode GroupNode ShapeNode ModelNode TextNode \
	   LightSource RayNode Contour ConstraintsNode \
	   GridNode PointerNode MeasurementNode SoundNode SoundSpace \
	   DSPNode Listener SoundConnection UserNode \
	   AnimationNode VideoTexture

ifdef SHARED_VIDEO_LIB
  #NODELIST += SharedVideoNode
  #NODELIST += SharedVideoTexture
endif

# turn nodelist into an object list of form: wrapper_<name>.o
WRAPPER_OBJ = $(patsubst %,wrapper_%.o,$(NODELIST))


#SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG -Wall
SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus -DASDEBUG


###################################################################
# GENERIC COMPILATION RULE:

wrapper_%.o: %.cpp
	$(CXX) $(SPIN_FLAGS) $(SPIN_INCLUDE) $(OSG_INCLUDE) $(BOOST_INCLUDE) -c $< -o $@

###################################################################
# TARGETS:

staticlib: $(WRAPPER_OBJ)
	$(AR) -rs $(LIBNAME).a $(WRAPPER_OBJ)

Linux: $(WRAPPER_OBJ)
	$(CXX) --export-dynamic -shared -Wl,-soname,$(LIBNAME).so -o $(LIBNAME).so $(WRAPPER_OBJ) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)

Darwin: $(WRAPPER_OBJ)
	$(CXX) -dynamiclib -undefined suppress -flat_namespace -o $(LIBNAME).dylib $(WRAPPER_OBJ) $(SPIN_LIBS) $(OSG_LIBS) $(BOOST_LIBS)


########################################################
install:
	@echo "\nINSTALLING to ${SPIN_INSTALL_PREFIX}";
	-cp $(LIBNAME).* ${SPIN_INSTALL_PREFIX}/lib

########################################################
clean:
	-rm -f $(LIBNAME)* *.o 

cleanb:
	-rm -f $(LIBNAME)* 
