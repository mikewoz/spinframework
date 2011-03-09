NAME=serializertest
CXX = g++
OBJS = myshape.o osgreflection.o myshape_wrapper.o
WRAPPER_OBJS = myshape_wrapper.o

all: $(NAME)


###################################################################

#CPPFLAGS += -I../../include -g -I/usr/include/boost $(shell python-config --cflags)
#LDFLAGS +=  -llo -lSPIN $(shell python-config --libs)
#LDFLAGS += -lpthread -lOpenThreads -losg -losgGA -losgSim -losgDB -losgFX \
		-losgIntrospection -losgManipulator -losgText -losgUtil \
		-losgParticle -losgViewer 

ifeq ($(shell uname -s),Darwin)
  LDFLAGS += -L/opt/local/lib `pkg-config --libs openscenegraph`
  CPPFLAGS += -I/opt/local/include `pkg-config --cflags openscenegraph`
else #linux
  LDFLAGS += -lc -lm -ldl -lGL -lX11 `pkg-config --libs openscenegraph`
  CPPFLAGS += -I/usr/local/include `pkg-config --cflags openscenegraph`
endif

# TARGETS:

%.o:%.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(CPPFLAGS) -c $< -o $@

$(NAME): $(NAME).o $(OBJS)
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(LDFLAGS)

wrapper: $(WRAPPER_OBJS)
	$(CXX) -shared -o $@ $^ $(LDFLAGS)

clean:
	-rm -f $(NAME) *.o
