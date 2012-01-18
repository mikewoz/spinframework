NAME=kinectViewer

all: $(NAME)

###################################################################

DEPS = cppintrospection-3.0 spin-0.3
CPPFLAGS += -g -I../../include -I../../src/tinyxml -I/usr/local/include/libfreenect -I/usr/include/boost $(shell python-config --cflags) `pkg-config --cflags $(DEPS)`
LDFLAGS += -lfreenect -llo $(shell python-config --libs) `pkg-config --libs $(DEPS)`
LDFLAGS +=  -lpthread -lOpenThreads -losg -losgGA -losgSim -losgDB -losgFX \
		-losgManipulator -losgText -losgUtil -losgSim -losgParticle -losgViewer

ifeq ($(shell uname -s),Darwin)
  CPPFLAGS += -I/opt/local/include
  LDFLAGS += -lboost_python -L/opt/local/lib
else #linux
  LDFLAGS += -lc -lm -ldl -lGL -lX11 -lboost_python
endif


# TARGETS:

%.o:%.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(CPPFLAGS) -c $< -o $@

$(NAME): $(NAME).o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(LDFLAGS)

install:
	-cp $(NAME) /usr/local/bin

clean:
	-rm -f $(NAME) *.o
