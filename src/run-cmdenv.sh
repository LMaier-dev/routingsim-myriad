#!/bin/bash
make -j8 MODE=debug && ./RoutingSim_dbg ../simulations/r2kademlia/eval.ini -u Cmdenv -c MANETRandomWaypoint -r 0 --cmdenv-express-mode=false  --cmdenv-log-level=trace # --cmdenv-redirect-output=true
# ./RoutingSim ../simulations/r2kademlia/eval.ini -u Cmdenv -c $* --cmdenv-redirect-output=true
