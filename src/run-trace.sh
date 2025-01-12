./RoutingSim_dbg ../simulations/KIRA/eval.ini -u Cmdenv -c ClosFatTree -r 2  --sim-time-limit=1000s --cmdenv-express-mode=false --cmdenv-log-level=trace 2>&1 | egrep -v 'DefaultNetwork\.manager\.Node[[[:xdigit:]]+\.node'
# useful extraction commands:
# filter out events from a specific node
#awk -e '/^\*\* Event.*DefaultNetwork\.manager\.Node49\.protocol0.*$/ { out = 1; print; next } out == 1 && /^[^\*]/ { print; } out == 1 && /^\*\* Event/ { out = 0; next }'
# filter out a specific time period
#sed -n '/^\*\*\ Event .* t=1\..*$/,/^\*\*\ Event .* t=1\.8.* .*$/p
#extract just the contact node ID
#sed -n 's/.*onNewContact(\(.*\)) called.*/\1/p'

#older stuff
#sed -n '/^\*\*\ Event.*DefaultNetwork\.manager\.Node3\.protocol0.*$/,/^\*\*\ Event/p'
#zcat ClosFatTree48-trace-20191213.log.gz | awk '/^\*\* Event .* DefaultNetwork\.manager\.Node8131\.protocol0/, /^\*\* Event .* DefaultNetwork\.manager\.Node[^8]+[^1]?[^3]?[^1]?.*/ { print $0; }'  >ClosFatTree48-Node8131-trace.log
# fgrep 'onNewContact(35' ClosFatTree-48-debug-2-27880.log | perl -n -e 'if (/.*onNewContact\(([[:xdigit:]]+)\)/) { print "$1\n"; }' | sort
#awk <debug-2000-old-n0-contacts.out 'match($2,/onNewContact\(([[:xdigit:]]{6})\)/,arr) { print arr[1] }' | sort
