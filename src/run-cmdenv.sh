#!/bin/bash
# argument is the configuration name, e.g., LinkFailureRandom, you can also add the run number with -r
nohup opp_runall -j40 ./RoutingSim ../simulations/KIRA/eval.ini -c $* -u Cmdenv --cmdenv-redirect-output=true > "$(HOSTNAME)_nohupRoutingSimOut.txt" &
# over view of runs
#./RoutingSim ../simulations/KIRA/eval.ini -u Cmdenv -c $* -q runs
# Full debug output (slow)
#./RoutingSim_dbg ../simulations/KIRA/eval.ini -u Cmdenv -c $* --cmdenv-express-mode=false  --cmdenv-log-level=trace --cmdenv-redirect-output=true
# Only errors
# ./RoutingSim ../simulations/KIRA/eval.ini -c MyStudy -r 0 -u Cmdenv --cmdenv-log-level=error --cmdenv-express-mode=false --cmdenv-event-banners=false
