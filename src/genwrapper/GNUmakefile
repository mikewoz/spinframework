EXEFILE = genwrapper

SRC_FILES = \
	main.cpp \
	NodeMap.cpp \
	RegistryBuilder.cpp \
	Configuration.cpp \
	Notify.cpp \
	TypeDesc.cpp \
	PathNameUtils.cpp \
	FileSystemUtils.cpp \
	TypeNameUtils.cpp \
	TypeRegistry.cpp \
	WrapperGenerator.cpp

OBJ_FILES = $(SRC_FILES:.cpp=.o)
ifeq ($(shell uname -s),Darwin)
	LIBS += -lxml2 -lc -lboost_regex-mt -L/opt/local/lib
else
	LIBS += -lxml2 -lc -lboost_regex -L/opt/local/lib
endif
CFLAGS += -I/usr/include/libxml2 -I/usr/include/boost -I/opt/local/include/boost-1_35 -I/opt/local/include

all: $(EXEFILE)

clean: 
	rm -f $(OBJ_FILES) $(EXEFILE)

$(EXEFILE): $(OBJ_FILES)
	${CXX} $(LIBS) $(LDFLAGS) -o $@ $(OBJ_FILES)

.cpp.o:
	${CXX} -Wall $(CFLAGS) -c $<

install:
	@echo "genwrapper will not be installed; the build system uses it locally";
