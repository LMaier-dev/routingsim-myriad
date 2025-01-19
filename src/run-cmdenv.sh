#!/bin/bash
# argument is the configuration name, e.g., LinkFailureRandom, you can also add the run number with -r
#nohup opp_runall -j16 ./RoutingSim ../simulations/KIRA/eval.ini -c $* -u Cmdenv --cmdenv-redirect-output=true > "$(hostname)_$*_nohupRoutingSimOut.txt" &
# over view of runs
#./RoutingSim ../simulations/KIRA/eval.ini -u Cmdenv -c $* -q runs
# Full debug output (slow)
#./RoutingSim_dbg ../simulations/KIRA/eval.ini -u Cmdenv -c $* --cmdenv-express-mode=false  --cmdenv-log-level=trace --cmdenv-redirect-output=true
# Only errors
# ./RoutingSim ../simulations/KIRA/eval.ini -c MyStudy -r 0 -u Cmdenv --cmdenv-log-level=error --cmdenv-express-mode=false --cmdenv-event-banners=false

config=kParameterStudieShort
runfilter="\$mN=2500 && \$mobilitySeed=0"

runs=$(opp_run ../simulations/KIRA/eval.ini -c ${config} -q runnumbers -s -r "${runfilter}")
for r in ${runs}; do
	opp_run ../simulations/KIRA/eval.ini -c ${config} -q runs -s -r${r}
	nohup ./RoutingSim ../simulations/KIRA/eval.ini -c ${config} -r${r} -u Cmdenv --cmdenv-redirect-output=true > "$(hostname)_${config}_${r}_nohupRoutingSimOut.txt" &
done
