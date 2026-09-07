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
# sum(ring lengths) = n + sharing surplus, so only two of {n, ring count, ring
# length} can be pinned at once. ringpipe spends the ring count on it (--hold-len
# solves --rings per point); ringgrow keeps the ring count and lets the length
# follow the identity mean length = (n + 2*tr - spof)/A, which is exact and so
# needs no solver.
#
# Every name carries the achieved figures, so sweep with:
#   eval_scaling.py --dir ./csv/<family>/ --x-key share
#   eval_scaling.py --dir ./csv/<family>/ --x-key tr
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
# budget at SPOF=0.5. Short rings and a wide tr span are therefore opposed
# (L ~ n/A + 1.5, span ~ (n/A - 2)/2.5). A=1200 is about as short as the rings
# go while the axis still spans anything: length 9.8..12.1 over a 2.5x tr range.
# --ring-size 4 is what makes the floor MEET that identity: the default 8 pads
# every ring carrying one or two transitions, which at this A costs ~100 nodes
# and pushes the top point over the target. It cannot change the achieved
# length -- padding to --target-n equalises the rings afterwards either way.
RINGS_TR=1200
TARGET_N_TR=10000
RING_SIZE_TR=4
SWEEP_TR="1200 1560 1920 2280 2640 3000"

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
    PY=ringgrow.py

    gen "full: transition sweep, A=${RINGS_TR} n=${TARGET_N_TR}" \
        --family delaunay --rings $RINGS_TR --sweep-tr $SWEEP_TR \
        --ring-size $RING_SIZE_TR --spof 0 --target-n $TARGET_N_TR &

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
        --spof 0 --ring-size 8 --target-n $TARGET_N &

    gen "full: share sweep at FIXED ring length, n=${TARGET_N}" \
        --family delaunay --rings $RINGS_BIG --width-max 6 --max-meet 12 \
        --sweep-share 0 0.49 0.65 0.74 0.84 0.915 0.875 0.952 --hold-len 16 \
        --spof $SPOF --ring-size 8 --target-n $TARGET_N
fi
