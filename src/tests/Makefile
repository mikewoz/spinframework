# First, we must provide the path to SPIN:
export SPIN_PATH = $(HOME)/src/spinframework/trunk

# the following file must be included:
include $(SPIN_PATH)/Makefile.include

all: testShaders

########################################################
## YOU SHOULDN'T NEED TO CHANGE ANYTHING BEYONG HERE! ##
########################################################

SPIN_FLAGS += -g -O2 -DC_PLUSPLUS -D__cplusplus

DEV_LDFLAGS = -L. -Wl,-rpath,. -L$(SPIN_PATH)/src/osgWrappers -Wl,-rpath,$(SPIN_PATH)/src/spin -Wl,-rpath,$(SPIN_INSTALL_PREFIX)/lib

SPIN_LIBS += -llo -lSPIN


########################################################
# GENERIC COMPILATION RULE

%.o:%.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(OSG_INCLUDE) $(SPIN_FLAGS) $(SPIN_INCLUDE) -c $< -o $@

########################################################
# TARGETS:

testShaders: testShaders.o Noise.o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(SPIN_LIBS) $(OSG_LIBS)


########################################################
clean:
	-rm -f *.o $(all)
