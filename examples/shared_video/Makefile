NAME=shared_video

all: $(NAME)

###################################################################

CPPFLAGS += -I../../include -g -I/usr/include/boost $(shell python-config --cflags)
LDFLAGS += -llo -lSPIN $(shell python-config --libs)
LDFLAGS += -lpthread -losg -losgGA -losgSim -losgDB -losgFX \
           -losgIntrospection -losgManipulator -losgText -losgUtil \
           -losgParticle -losgViewer -lc -lm -ldl -lGL -lX11 -lboost_python

# TARGETS:

%.o:%.cpp
	@echo "--------------- compiling $@ ---------------" 
	$(CXX) $(CPPFLAGS) -c $< -o $@

$(NAME): $(NAME).o
	@echo "\n--------------- linking $@: ---------------" 
	$(CXX) -o $@ $^ $(LDFLAGS)

install:
	-cp $(NAME) /usr/local/bin

.PHONY: clean

clean:
	-rm -f $(NAME) *.o
