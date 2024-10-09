#!/bin/bash
# argument is the configuration name, e.g., LinkFailureRandom, you can also add the run number with -r
./RoutingSim ../simulations/KIRA/eval.ini -c $* -u Cmdenv --cmdenv-redirect-output=true
# over view of runs
#./RoutingSim ../simulations/KIRA/eval.ini -u Cmdenv -c $* -q runs
# Full debug output (slow)
#./RoutingSim_dbg ../simulations/KIRA/eval.ini -u Cmdenv -c $* --cmdenv-express-mode=false  --cmdenv-log-level=trace --cmdenv-redirect-output=true
