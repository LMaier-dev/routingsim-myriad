# RoutingSim

This is RoutingSim, an event-discrete simulator for routing protocols based on 
OMNet++.

The paper explaining the idea behind RoutingSim can be found here:
Werle, C.; Mies, S.; Zitterbart, M.: On Benchmarking Routing Protocols, 2011. 
17th IEEE International Conference on Networks (ICON 2011), 
Singapore, SGP, December 14-16, 2011, 305–310, IEEE, Piscataway, NJ. 
doi:10.1109/ICON.2011.6168493 
https://ieeexplore.ieee.org/document/6168493

The current version has been tested with OMNeT++ 6.0.3 and OMNeT++ 5.7.

## Installation
While OMNeT++ is able to run under Windows, the recommended and
tested environment is a native Linux installation (e.g., Ubuntu)
Get OMNeT++ from https://omnetpp.org/ and install it according to
the manual. You should make sure that
`CXXFLAGS=-std=c++20`
is set in `configure.user` of OMNeT++.

Note: you should use at least g++-12 as g++-11 has a compiler bug
that causes crashes.

Additionally, one needs boost, e.g., libboost-all-dev
as well as libgmp (libgmp-dev).

Once you have OMNeT++ in your PATH and LD_LIBRARY_PATH,
type
`make makefiles`
to generate the makefiles and
afterwards 
`make -j 8`
or `make -j 8 MODE=debug`
to create a debug built
(`-j 8` starts 8 compile processes in parallel, choose according to
you number of available cores, just using `-j` tries to do this automatically, but
it may overload your machine, esp. with respect to RAM consumption).

Create a symbolic link in src/ to the topologies directory:

    cd src/
	ln -s ../simulations/topologies .

To start a simulation you can change to src/
and type in

    ./RoutingSim_dbg ../simulations/KIRA/eval.ini

Simulations for evaluations should run with the Cmdenv interface
and compiled with MODE=release.

Result files are stored in ../simulations/KIRA/results
for the example above.

## State Export
The state export functionality allows to dump the contents of all routingtables at the end of a simulation for offline analysis.

### Dependencies
For the state-export functionality one needs [https://github.com/nlohmann/json](https://github.com/nlohmann/json) version 3.10.5:

- The library is also available in some package managers. In that case you can install it and skip the rest.
  - Fedora: [json-devel](https://packages.fedoraproject.org/pkgs/json/json-devel/)
  - Arch (AUR): [nlohman-json-git](https://aur.archlinux.org/packages/nlohmann-json-git)
  - Debian (sid):[nlohmann-json3-dev](https://packages.debian.org/sid/nlohmann-json3-dev), e.g., ```apt install nlohmann-json3-dev```
  - Ubuntu (jammy): [nlohmann-json3-dev](https://packages.ubuntu.com/jammy/nlohmann-json3-dev) 
- [Download](https://github.com/nlohmann/json/releases/download/v3.10.5/include.zip)
  - Extract somewhere
  - Set env var to single_include directory in extracted folder
  ```
  export JSON_INCL_DIR=/path/to/extracted/folder/single_include
  ```
  - Then run make makefiles


### Feature Flag
Activate the compiletime flag during compilation: ```make -j8 MODE=release CFLAGS="-DR2K_RT_EXPORT"```

### Activate in ```eval.ini```:
```ini
**.dumpRTState = true
```

Hint: Currently it is required to explicitly set the result folder using the cli option: ```--result-dir=`pwd`/../simulations/KIRA/results```
