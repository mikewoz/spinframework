include ./Makefile.include

# note: order is important:
all: wrappers libAudioscape doc vess

doc:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating Documentation <<<<<<<<<<<<<<<<<<<<<"; \
	doxygen ./doxygen_config	

wrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating Wrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(GENWRAPPER_BIN) -d . doxygen | doxygen -
	$(GENWRAPPER_BIN) -v DEBUG doxygen .
	
libAudioscape:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building libAudioscape <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers

vess:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Building VESS <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/vess

clean:
	-rm src/osgWrappers/*.cpp
	$(MAKE) -C src/osgWrappers clean
	$(MAKE) -C src/vess clean


