#!/usr/bin/env bash
# Generate the MYRIAD topology families, 5 seeds each.
#
# Two axes, each holding n fixed:
#   --sweep-share     fan-out (rings meeting at one interface); ring length ~flat
#   --sweep-ring-len  ring length (lap length); --rings is solved per point
#
# Both axes come in a plain and a HELD variant. Plain lets the other quantity
# drift; held pins it, so exactly one thing moves:
#   --hold-len  on a share sweep  -> ring length pinned, --rings solved per point
#   --hold-fan  on a length sweep -> fan-out pinned, --share solved per point
# sum(ring lengths) = n + sharing surplus, so only two of {n, ring count, ring
# length} can be pinned at once -- the held variants spend the third.
#
# Each axis gets a QUICK family (n~100-200, seconds) for smoke tests and a FULL
# family for real runs. Every name carries the achieved median ring length as
# _rl<N>, so sweep with:
#   eval_scaling.py --dir ./csv/<family>/ --x-key share
#   eval_scaling.py --dir ./csv/<family>/ --x-key rl
# eval_scaling averages the seeds of each point automatically (same name bar
# _seed<N>), so a plotted curve is a mean rather than one lucky draw.
set -euo pipefail

TOPO_DIR=/root/routingsim/simulations/MYRIAD/myriad_topologies
SPOF=0.25
SEEDS="0 1 2 3 4"

RINGS_BIG=150            # share sweep: --rings is fixed, n held by padding
TARGET_N=1000
RINGS_QUICK=16           # floors at 98 nodes, so n=100 is reachable at every share
TARGET_N_QUICK=100

# The length axis needs NARROW interfaces: a ring carrying several interfaces of
# width w can never be shorter than their total, so wide ones pin the short end.
# width-max 2 with share 0 is what makes 8..32 actually reachable.
LEN_WIDTH=2
TARGET_N_LEN=1000
TARGET_N_LEN_QUICK=200

LOG_DIR=$(mktemp -d)
trap 'rm -rf "$LOG_DIR"' EXIT

wipe() {   # a family is (csv, ringlabels, gallery); regenerate all three together
    for d in ringlabels gallery csv; do rm -rf "${TOPO_DIR}/${d}/$1"; done
}

# Seed 0 runs in the foreground so there is live progress; the other seeds run
# beside it on their own cores, logging to files. Nothing starts the next family
# until all of them are done, since they would otherwise fight for the machine.
gen() {
    local label=$1; shift
    echo "=== $label (seeds: $SEEDS) ==="
    for s in ${SEEDS#* }; do
        python3 "$TOPO_DIR/ringpipe.py" "$@" --seed "$s" > "$LOG_DIR/seed$s.log" 2>&1 &
    done
    python3 "$TOPO_DIR/ringpipe.py" "$@" --seed "${SEEDS%% *}"
    wait
    echo "  (other seeds done: $(ls "$LOG_DIR" | wc -l) logs)"
    rm -f "$LOG_DIR"/seed*.log
}

wipe "delaunay_A${RINGS_BIG}_spof${SPOF}"
wipe "delaunay_A${RINGS_QUICK}_spof${SPOF}"
wipe "delaunay_len${TARGET_N_LEN}_spof${SPOF}"
wipe "delaunay_len${TARGET_N_LEN_QUICK}_spof${SPOF}"
# the held variants name their folder from the anchor they solved for, so they
# are wiped by glob rather than by an exact name
rm -rf "${TOPO_DIR}"/{csv,ringlabels,gallery}/delaunay_shr*_spof${SPOF}
rm -rf "${TOPO_DIR}"/{csv,ringlabels,gallery}/delaunay_lenfan*_spof${SPOF}

gen "quick: share sweep, n=${TARGET_N_QUICK}" \
    --family delaunay --rings $RINGS_QUICK --width-max 6 \
    --sweep-share 0.0 0.25 0.5 0.75 1.0 \
    --spof $SPOF --ring-size 8 --target-n $TARGET_N_QUICK

gen "quick: ring-length sweep, n=${TARGET_N_LEN_QUICK}" \
    --family delaunay --width-max $LEN_WIDTH --share 0 \
    --sweep-ring-len 8 12 16 \
    --spof $SPOF --target-n $TARGET_N_LEN_QUICK

gen "full: share sweep at FIXED ring length, n=${TARGET_N}" \
    --family delaunay --rings $RINGS_BIG --width-max 6 \
    --sweep-share 0.0 0.2 0.4 0.6 0.8 1.0 --hold-len \
    --spof $SPOF --ring-size 8 --target-n $TARGET_N

gen "full: ring-length sweep at FIXED fan-out, n=${TARGET_N_LEN}" \
    --family delaunay --width-max $LEN_WIDTH \
    --sweep-ring-len 4 8 12 16 24 32 --hold-fan \
    --spof $SPOF --target-n $TARGET_N_LEN
