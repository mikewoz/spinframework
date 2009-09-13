

# this lists the binaries to produce, the (non-PHONY, binary) targets in
# the previous manual Makefile
bin_PROGRAMS = genwrapper
genwrapper_SOURCES = main.cpp Configuration.cpp FileSystemUtils.cpp NodeMap.cpp Notify.cpp PathNameUtils.cpp RegistryBuilder.cpp TypeDesc.cpp TypeNameUtils.cpp TypeRegistry.cpp WrapperGenerator.cpp

genwrapper_CPPFLAGS = $(XML_CFLAGS) $(BOOST_CPPFLAGS)
genwrapper_LDFLAGS = $(XML_LIBS) $(BOOST_LDFLAGS)
#genwrapper_LDFLAGS = $(XML_LIBS) $(BOOST_LDFLAGS) -l$(BOOST_REGEX_LIB)
genwrapper_LDADD = $(BOOST_REGEX_LIB)

