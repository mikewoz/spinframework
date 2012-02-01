NAME=spacenavigator

all: $(NAME)

LDFLAGS += -weak_framework 3DconnexionClient
#LDFLAGS += -framework 3DconnexionClient
#LDFLAGS += -framework Carbon

###################################################################

DEPS = cppintrospection-3.0 spin-0.3

CPPFLAGS += -I../../include -g -I/usr/include/boost `pkg-config --cflags $(DEPS)` $(shell python-config --cflags)
LDFLAGS +=  -llo `pkg-config --libs $(DEPS)` $(shell python-config --libs)
LDFLAGS +=  -lpthread -lOpenThreads -losg -losgGA -losgSim -losgDB -losgFX \
		-losgManipulator -losgText -losgUtil \
		-losgParticle -losgViewer

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
