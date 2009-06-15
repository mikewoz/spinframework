include ./Makefile.include

# note: order is important:
all: genwrapper osgWrappers libSPIN vess

genwrapper:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building genwrapper <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/genwrapper

doc:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating Documentation <<<<<<<<<<<<<<<<<<<<<"; \
	doxygen ./doxygen_config	

osgWrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating osgWrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(GENWRAPPER_BIN) -d . doxygen | doxygen -
	$(GENWRAPPER_BIN) -v QUIET -c genwrapper.conf doxygen .
	
libSPIN:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building libSPIN <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers

vess:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building VESS <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/vess

clean:
	-rm src/osgWrappers/*.cpp
	$(MAKE) -C src/osgWrappers clean
	$(MAKE) -C src/vess clean


