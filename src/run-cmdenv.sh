#!/bin/bash
# argument is the configuration name, e.g., LinkFailureRandom, you can also add the run number with -r
./RoutingSim ../simulations/r2kademlia/eval.ini -u Cmdenv -c $* --cmdenv-redirect-output=true
#./RoutingSim_dbg ../simulations/r2kademlia/eval.ini -u Cmdenv -c $* --cmdenv-express-mode=false  --cmdenv-log-level=debug --cmdenv-redirect-output=true
