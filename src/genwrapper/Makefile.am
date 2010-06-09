#AUTOMAKE_OPTIONS = foreign
#ACLOCAL_AMFLAGS = -I ../../m4

# this lists the binaries to produce, the (non-PHONY, binary) targets in
# the previous manual Makefile
bin_PROGRAMS = genwrapper
genwrapper_SOURCES = main.cpp Configuration.cpp FileSystemUtils.cpp NodeMap.cpp Notify.cpp PathNameUtils.cpp RegistryBuilder.cpp TypeDesc.cpp TypeNameUtils.cpp TypeRegistry.cpp WrapperGenerator.cpp Configuration.h FileSystemUtils.h NodeMap.h Notify.h PathNameUtils.h RegistryBuilder.h TypeDesc.h TypeNameUtils.h TypeRegistry.h WrapperGenerator.h XML_node.h XML_parser.h



genwrapper_CPPFLAGS = $(XML_CFLAGS) $(BOOST_CPPFLAGS)
genwrapper_LDFLAGS = $(XML_LIBS) $(BOOST_LDFLAGS) $(BOOST_REGEX_LIB)

