# OMNET executables need to be in the PATH opp_makemake
include $(shell opp_configfilepath)

BUILD_OPTIONS = -f --deep -o RoutingSim -I. -Ifoundation -I$(OMNETPP_INCL_DIR)/omnetpp -I$(OMNETPP_INCL_DIR) -DAUTOIMPORT_OMNETPP_NAMESPACE -lgmp

all: checkmakefiles
	cd src && $(MAKE)

clean: checkmakefiles
	cd src && $(MAKE) clean

cleanall: checkmakefiles
	cd src && $(MAKE) MODE=release clean
	cd src && $(MAKE) MODE=debug clean
	rm -rf src/Makefile
	rm -rf out

makefiles:
	cd src && opp_makemake $(BUILD_OPTIONS)

checkmakefiles:
	@if [ ! -f src/Makefile ]; then \
	echo; \
	echo '======================================================================='; \
	echo 'src/Makefile does not exist. Please use "make makefiles" to generate it!'; \
	echo '======================================================================='; \
	echo; \
	exit 1; \
        fi

doxy:
	doxygen doxy.cfg

TAGS:
	find . -name "*.h" | etags -o TAGS --language=c++ -
	find . -name "*.cc" | etags -o TAGS --language=c++ -a -

topologies: plcTopo fatTreeTopo

-include topomake

.PHONY: checkmakefiles makefiles clean cleanall doxy TAGS

