#!/usr/bin/env bash
# Run every eval in eval_runs.txt in its own tmux window.
#
#   ./run_evals.sh [runfile]            start, then attach
#   ATTACH=0 ./run_evals.sh             start detached (ssh/CI)
#   PIN=1 ./run_evals.sh                pin each run to its own core
#
# Each window logs to eval/logs/<n>_<slug>.log and stays open after the run so
# the exit status is readable. Windows are independent processes, so the OS
# already spreads them across cores; PIN=1 only matters if you want to stop runs
# migrating between cores or reserve capacity for something else.
set -euo pipefail

MYRIAD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNFILE="${1:-$MYRIAD_DIR/eval_runs.txt}"
WORKDIR="$(cd "$MYRIAD_DIR/../../src" && pwd)"   # runs use ../simulations/... paths
LOGDIR="$MYRIAD_DIR/eval/logs"
SESSION="${SESSION:-myriad-eval}"

[[ -f "$RUNFILE" ]] || { echo "no runfile: $RUNFILE" >&2; exit 1; }
command -v tmux >/dev/null || { echo "tmux not installed" >&2; exit 1; }
tmux has-session -t "$SESSION" 2>/dev/null && {
    echo "session '$SESSION' already exists — attach with: tmux attach -t $SESSION" >&2
    echo "(or kill it: tmux kill-session -t $SESSION)" >&2
    exit 1
}

mkdir -p "$LOGDIR"
NCPU=$(getconf _NPROCESSORS_ONLN)

# tmux windows inherit the tmux *server's* environment, not the caller's, so
# sourcing setenv before running this script does not reach them. Resolve
# OMNeT++'s bin dir here and push it into each window; without opp_scavetool the
# evals still produce their tables but skip the byte-rate plots.
if ! command -v opp_scavetool >/dev/null; then
    for d in "${OMNETPP_ROOT:-}" "$HOME/omnetpp-6.3.0" /opt/omnetpp-6.3.0; do
        [[ -x "$d/bin/opp_scavetool" ]] && { export PATH="$d/bin:$PATH"; break; }
    done
    command -v opp_scavetool >/dev/null \
        || echo "note: opp_scavetool not found — byte-rate plots will be skipped" >&2
fi

mapfile -t RUNS < <(grep -vE '^\s*(#|$)' "$RUNFILE")
(( ${#RUNS[@]} )) || { echo "no runs in $RUNFILE" >&2; exit 1; }
(( ${#RUNS[@]} <= NCPU )) || echo "note: ${#RUNS[@]} runs on $NCPU cores — they will share" >&2

# The oracle gate writes shared scratch under eval/oracle/, so letting every run
# start it at once interleaves the RIB dumps (KeyError: 'cost'). Run it once here;
# it stamps the binary and each eval below then skips it. GATE=0 to skip entirely.
if [[ "${GATE:-1}" == 1 ]]; then
    echo "running oracle gate once (serial) …"
    ( cd "$WORKDIR" && python3 "$MYRIAD_DIR/eval_entrypoint.py" gate ) \
        2>&1 | tee "$LOGDIR/gate.log" | tail -3
    # tee hides the exit status; PIPESTATUS[0] is the gate's.
    (( ${PIPESTATUS[0]} == 0 )) || { echo "oracle gate FAILED — see $LOGDIR/gate.log" >&2; exit 1; }
fi

i=0
for cmd in "${RUNS[@]}"; do
    # Window name from the distinguishing options, so tabs are tellable apart:
    # script, topology stem, and the flags that actually vary between runs.
    # `|| true` on each: grep exits 1 when a run has no --topo/--only, and
    # set -e would abort the whole script on that.
    # eval_entrypoint is every dynamics run, so it distinguishes nothing.
    script=$(sed -E 's#.*eval_([a-z_]+)\.py.*#\1#; s#^entrypoint$##' <<<"$cmd")
    topo=$(grep -oE '\-\-topo +\S+' <<<"$cmd" | sed -E 's#.*/##; s/\.txt//') || true
    # Node count first, so the tabs sort by scale at a glance; drop it from the
    # stem so the basis/scenario survives the length cut below.
    size=$(grep -oE '_n[0-9]+' <<<"$topo" | head -1 | tr -d '_') || true
    [[ -n "$size" ]] && topo=${topo/_$size/}
    opts=$(grep -oE '\-\-(only|basis) +\S+' <<<"$cmd" | awk '{print $2}' | paste -sd_ -) || true
    slug=$(sed -E 's/[^a-zA-Z0-9]+/_/g; s/^_|_$//g' \
           <<<"${size}_${script}_${topo}_${opts}" | cut -c1-32)
    name="$(printf '%02d' $i)_${slug}"
    log="$LOGDIR/$name.log"

    run="$cmd"
    if [[ "$cmd" == SPLIT\ * ]]; then
        # SPLIT <dynamics run>: one parallel process per config the run's flags
        # select (gp, gp_nospof, ls, …) into the same topo-scoped output dir,
        # then --post re-reads them all and draws the combined plots. Asking the
        # eval for the list keeps this in step with --nospof/--noreplay instead
        # of hardcoding the families. The pre-run oracle gate above is already
        # stamped, so every child skips it.
        # ponytail: `wait` returns the last job's status, so a mid-run failure
        # still surfaces in the log and in --post's "run this config first" error.
        base="${cmd#SPLIT }"
        mapfile -t cfgs < <(cd "$WORKDIR" && $base --list-configs 2>/dev/null)
        (( ${#cfgs[@]} )) || cfgs=(gp ls)   # older eval, or the query failed
        run=""
        for c in "${cfgs[@]}"; do run+="$base --protos $c & "; done
        run="( $run wait ) && $base --post"
    elif [[ "${PIN:-0}" == 1 ]]; then
        run="taskset -c $(( i % NCPU )) $cmd"
    fi
    # The run goes into a temp script rather than being interpolated into the
    # tmux command string: quoting survives one fewer shell that way, and a run
    # line with quotes/globs reaches python exactly as written in the runfile.
    # `exec bash` keeps the pane alive after the run so its tail is readable.
    body_sh="$LOGDIR/.$name.sh"
    { printf '#!/usr/bin/env bash\nexport PATH=%q\ncd %q || exit 1\n' "$PATH" "$WORKDIR"
      printf 'printf "$ %%s\\n" %q | tee %q\n' "$cmd" "$log"
      printf '{ %s; } 2>&1 | tee -a %q\n' "$run" "$log"
      printf 'echo; echo "[exit ${PIPESTATUS[0]}] log: %s"\nexec bash\n' "$log"
    } > "$body_sh"
    chmod +x "$body_sh"
    body="$body_sh"

    if (( i == 0 )); then
        tmux new-session -d -s "$SESSION" -n "$name" "$body"
    else
        tmux new-window -t "$SESSION" -n "$name" "$body"
    fi
    i=$((i + 1))   # not ((i++)): that returns 1 when i was 0, which set -e traps
done

tmux select-window -t "$SESSION:^"    # first window, whatever base-index is set
echo "started $i run(s) in tmux session '$SESSION'  (logs: $LOGDIR)"
tmux list-windows -t "$SESSION" -F '  #{window_index}: #{window_name}'
[[ "${ATTACH:-1}" == 1 ]] && exec tmux attach -t "$SESSION"
echo "attach with: tmux attach -t $SESSION"
