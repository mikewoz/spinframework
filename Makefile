include ./Makefile.include

# note: order is important:
all: genwrapper osgWrappers libSPIN spin

genwrapper:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building genwrapper <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/genwrapper

doc:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating Documentation <<<<<<<<<<<<<<<<<<<<<"; \
	doxygen ./doxygen_config	
	$(MAKE) -C src/spin doc

osgWrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating osgWrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(GENWRAPPER_BIN) -d . doxygen | doxygen -
	$(GENWRAPPER_BIN) -v QUIET -c genwrapper.conf doxygen .
	
libSPIN:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building libSPIN <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers

spin:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building SPIN <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/spin

clean:
	-rm -rf doxygen/xml
	-rm src/osgWrappers/*.cpp
	-rm -rf doxygen/xml
	$(MAKE) -C src/osgWrappers clean
	$(MAKE) -C src/spin clean

cleanb:
	$(MAKE) -C src/osgWrappers cleanb
	$(MAKE) -C src/spin cleanb

install:
	$(MAKE) -C src/osgWrappers install
	$(MAKE) -C src/spin install

