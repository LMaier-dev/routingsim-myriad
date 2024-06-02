#!/bin/bash
make -j8 && ./RoutingSim ../simulations/r2kademlia/eval.ini -u Cmdenv -c MANETRandomWaypoint $*
