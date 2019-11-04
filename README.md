This is RoutingSim, an event-discrete simluator for routing protocols based on 
OMNet++.

The current version has been tested with OMNeT++ 5.5.1 (and 5.4.1 as well).

== Installation ==
Get OMNeT++ from https://omnetpp.org/ and install it according to
the manual. Once you have OMNeT++ in your PATH and LD_LIBRARY_PATH,
type
{{{ make makefiles }}}
to generate the makefiles and
afterwards 
{{{ make }}}
or {{{ make -j 4 MODE=debug }}}
to create a debug built.

To start a simulation you can change to src/
and type in
{{{ ./RoutingSim_dbg ../simulations/r2kademlia/thesis.ini }}}
Simulations for evaluations should run with the Cmdenv interface
and compiled with MODE=release.

