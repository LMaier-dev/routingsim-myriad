This is RoutingSim, an event-discrete simulator for routing protocols based on 
OMNet++.

The paper explaining the idea behind RoutingSim can be found here:
Werle, C.; Mies, S.; Zitterbart, M.: On Benchmarking Routing Protocols, 2011. 
17th IEEE International Conference on Networks (ICON 2011), 
Singapore, SGP, December 14-16, 2011, 305–310, IEEE, Piscataway, NJ. 
doi:10.1109/ICON.2011.6168493 
https://ieeexplore.ieee.org/document/6168493

The current version has been tested with OMNeT++ 5.5.1 (and 5.4.1 as well).

# Installation
Get OMNeT++ from https://omnetpp.org/ and install it according to
the manual. Once you have OMNeT++ in your PATH and LD_LIBRARY_PATH,
type
`make makefiles`
to generate the makefiles and
afterwards 
`make`
or `make -j 4 MODE=debug`
to create a debug built.

To start a simulation you can change to src/
and type in
`./RoutingSim_dbg ../simulations/r2kademlia/thesis.ini`
Simulations for evaluations should run with the Cmdenv interface
and compiled with MODE=release.
