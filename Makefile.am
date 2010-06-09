# "foreign" means not GNU, and is common for avoiding boring messages about files organized differently from what gnu expects
AUTOMAKE_OPTIONS = foreign

ACLOCAL_AMFLAGS = -I m4

SUBDIRS = src

EXTRA_DIST = README LICENSE

wrappers:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating osgWrappers <<<<<<<<<<<<<<<<<<<<<"; \
	$(MAKE) -C src/osgWrappers wrappers

doc:
	@echo ""; \
	echo ">>>>>>>>>>>>>>>>>>>>> Generating Documentation <<<<<<<<<<<<<<<<<<<<<"; \
	doxygen ./doxygen_config        
	$(MAKE) -C src/spin doc

INCLUDES = src/tinyxml
