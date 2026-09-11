#!/usr/bin/env bash
# Generate the MYRIAD topology families, 5 seeds each.
#
# Two axes, each holding n fixed:
#   ringpipe --sweep-share  fan-out (rings meeting at one interface), with
#                           --hold-len pinning ring length so only fan moves
#   ringgrow --sweep-tr     transitions (ring pairs that meet), at a fixed ring
#                           AND node count; every interface is a plain two-node
#                           corridor, so nothing has to be solved for
#
# and one that moves n instead:
#   ringgrow --sweep-n      NODE count, at a fixed ring and transition count.
#                           fan = 4*tr - 2*spof, so the ring graph stays put and
#                           only n (and the length that follows it) moves -- the
#                           axis per-router state and link load scale on
#
# sum(ring lengths) = n + sharing surplus, so only two of {n, ring count, ring
# length} can be pinned at once. ringpipe spends the ring count on it (--hold-len
# solves --rings per point); ringgrow keeps the ring count and lets the length
# follow the identity mean length = (n + 2*tr - spof)/A, which is exact and so
# needs no solver.
#
# Every name carries the achieved figures, so sweep with:
#   eval_scaling.py --dir ./csv/<family>/ --x-key share
#   eval_scaling.py --dir ./csv/<family>/ --x-key tr
#   eval_scaling.py --dir ./csv/<family>/ --x-key n
# Every topology is its own point on those plots, seeds included, so a curve is
# a regression over the spread rather than one lucky draw.
#   generate_topos.sh [grow|pipe|all]     default all; each family is wiped and
#                                         regenerated only if it is selected
set -euo pipefail

WHICH=${1:-all}
case $WHICH in
    grow|pipe|all) ;;
    *) echo "usage: $0 [grow|pipe|all]" >&2; exit 2 ;;
esac

TOPO_DIR=~/routingsim/simulations/MYRIAD/myriad_topologies
SPOF=0.5
SEEDS="0 1 2 3 4"
# Two levels of parallelism multiply: the seeds of a family run side by side,
# and each of those spreads its own sweep points over --jobs processes. Sized so
# seeds*JOBS lands near the core count.
JOBS=${JOBS:-$(( ( $(nproc) + $(set -- $SEEDS; echo $#) - 1 ) / $(set -- $SEEDS; echo $#) ))}

RINGS_BIG=150            # share sweep: --rings is fixed, n held by padding
TARGET_N=1000
RINGS_QUICK=16           # floors at 98 nodes, so n=100 is reachable at every share
TARGET_N_QUICK=100

# Transition sweep. Ring length is not a knob here: mean length = (n + 2*tr -
# spof)/A, so A alone sets it, and A also squeezes the sweep from both ends --
# tr >= A-1 keeps the ring graph connected, tr <= (n - 2*A)/2.5 is the node
# budget at SPOF=0.5. n=2500 at A=300 keeps rings 9.8..11.8 long, the same band
# the n=10000 family had, so it is that experiment at a quarter of the scale --
# and a quarter-scale run fits in memory many times over, which is what lets the
# eval lines all run at once instead of in hand-uncommented phases.
# --ring-size 4 makes the floor MEET the identity: the default 8 pads every ring
# carrying one or two transitions, and that slack pushes the top point over the
# target. It cannot change the achieved length -- padding to --target-n
# equalises the rings afterwards either way.
RINGS_TR=300
TARGET_N_TR=2500
RING_SIZE_TR=4
SWEEP_TR="300 400 500 600 700"

TR_N=750
SWEEP_N="3000 4000 5000 6000 8000"

RINGS_SMALL=60
TARGET_N_SMALL=500
SWEEP_TR_SMALL="60 80 100 120 140"

LOG_DIR=$(mktemp -d)
trap 'rm -rf "$LOG_DIR"' EXIT

wipe() {   # a family is (csv, ringlabels, gallery); regenerate all three together
    for d in ringlabels gallery csv; do rm -rf "${TOPO_DIR}/${d}/$1"; done
}

# Seed 0 runs in the foreground so there is live progress; the other seeds run
# beside it on their own cores, logging to files. Nothing starts the next family
# until all of them are done, since they would otherwise fight for the machine.
PY=ringpipe.py           # generator gen() drives; switched per family below
gen() {
    local label=$1; shift
    echo "=== $label (seeds: $SEEDS, --jobs $JOBS each) ==="
    for s in ${SEEDS#* }; do
        python3 "$TOPO_DIR/$PY" "$@" --jobs "$JOBS" --seed "$s" \
            > "$LOG_DIR/seed$s.log" 2>&1 &
    done
    python3 "$TOPO_DIR/$PY" "$@" --jobs "$JOBS" --seed "${SEEDS%% *}"
    wait
    echo "  (other seeds done: $(ls "$LOG_DIR" | wc -l) logs)"
    rm -f "$LOG_DIR"/seed*.log
}

if [[ $WHICH != pipe ]]; then
    wipe "delaunay_grow${TARGET_N_TR}_A${RINGS_TR}_spof0"
    wipe "delaunay_grow${TARGET_N_TR}_A${RINGS_TR}_spof${SPOF}"
    wipe "delaunay_nsweep_A${RINGS_TR}_tr${TR_N}_spof${SPOF}"
    wipe "delaunay_grow${TARGET_N_SMALL}_A${RINGS_SMALL}_spof${SPOF}"
    PY=ringgrow.py

    gen "quick: transition sweep, A=${RINGS_SMALL} n=${TARGET_N_SMALL}" \
        --family delaunay --rings $RINGS_SMALL --sweep-tr $SWEEP_TR_SMALL \
        --ring-size $RING_SIZE_TR --spof $SPOF --target-n $TARGET_N_SMALL

    gen "full: transition sweep, A=${RINGS_TR} n=${TARGET_N_TR}" \
        --family delaunay --rings $RINGS_TR --sweep-tr $SWEEP_TR \
        --ring-size $RING_SIZE_TR --spof 0 --target-n $TARGET_N_TR &

    gen "full: transition sweep, A=${RINGS_TR} n=${TARGET_N_TR}" \
        --family delaunay --rings $RINGS_TR --sweep-tr $SWEEP_TR \
        --ring-size $RING_SIZE_TR --spof $SPOF --target-n $TARGET_N_TR

    gen "full: node sweep, A=${RINGS_TR} tr=${TR_N}" \
        --family delaunay --rings $RINGS_TR --tr $TR_N --sweep-n $SWEEP_N \
        --ring-size $RING_SIZE_TR --spof $SPOF

    RINGS_TR=1200
    TARGET_N_TR=10000
    SWEEP_TR="1200 1560 1920 2280 2640 3000"
    
    gen "full: transition sweep, A=${RINGS_TR} n=${TARGET_N_TR}" \
        --family delaunay --rings $RINGS_TR --sweep-tr $SWEEP_TR \
        --ring-size $RING_SIZE_TR --spof $SPOF --target-n $TARGET_N_TR
fi

if [[ $WHICH != grow ]]; then
    wipe "delaunay_A${RINGS_BIG}_spof${SPOF}"
    wipe "delaunay_A${RINGS_QUICK}_spof${SPOF}"
    # the held variants name their folder from the anchor they solved for, so
    # they are wiped by glob rather than by an exact name
    rm -rf "${TOPO_DIR}"/{csv,ringlabels,gallery}/delaunay_shr*_spof${SPOF}
    PY=ringpipe.py

    gen "full: share sweep at FIXED ring length, n=${TARGET_N}" \
        --family delaunay --rings $RINGS_BIG --width-max 6 --max-meet 12 \
        --sweep-share 0 0.49 0.65 0.74 0.84 0.915 0.875 0.952 --hold-len 16 \
        --spof $SPOF --ring-size 8 --target-n $TARGET_N
fi
