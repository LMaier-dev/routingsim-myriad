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
    if ( cd "$WORKDIR" && python3 "$MYRIAD_DIR/eval_entrypoint.py" gate ) \
        2>&1 | tee "$LOGDIR/gate.log" | tail -3; rc=${PIPESTATUS[0]}; (( rc != 0 )); then
        echo "oracle gate FAILED (exit $rc) — see $LOGDIR/gate.log" >&2
        exit 1
    fi
fi

# A pane runs a generated script rather than an interpolated tmux command
# string: quoting survives one fewer shell that way, so a run line with
# quotes/globs reaches python exactly as written in the runfile. `exec bash`
# keeps the pane alive after the run so its tail is readable.
#   mkbody <script-path> <log> <banner> <command>
mkbody() {
    { printf '#!/usr/bin/env bash\nexport PATH=%q\ncd %q || exit 1\n' "$PATH" "$WORKDIR"
      printf 'printf "$ %%s\\n" %q | tee %q\n' "$3" "$2"
      printf '{ %s; } 2>&1 | tee -a %q\n' "$4" "$2"
      printf 'echo; echo "[exit ${PIPESTATUS[0]}] log: %s"\nexec bash\n' "$2"
    } > "$1"
    chmod +x "$1"
}

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

    # SPLIT <dynamics run>: one pane per config the run's flags select (gp,
    # gp_nospof, ls, …), all writing the same topo-scoped output dir, plus a
    # final pane that waits for them and runs --post to draw the combined
    # plots. Asking the eval for the list keeps this in step with
    # --nospof/--noreplay instead of hardcoding the families. The pre-run
    # oracle gate above is already stamped, so every child skips it.
    cfgs=()
    if [[ "$cmd" == SPLIT\ * ]]; then
        base="${cmd#SPLIT }"
        mapfile -t cfgs < <(cd "$WORKDIR" && eval "$base --list-configs" 2>/dev/null)
        if (( ! ${#cfgs[@]} )); then       # older eval, or the query failed
            echo "WARNING: --list-configs gave nothing for:" >&2
            echo "  $base" >&2
            echo "  falling back to (gp ls) — any other config gets no pane," >&2
            echo "  and --post will then fail on the first one missing." >&2
            cfgs=(gp ls)
        fi
    fi

    if (( ${#cfgs[@]} )); then
        # Panes are separate processes, so `wait` can't join them: each config
        # pane drops a file on exit and the post pane counts them. Dropped
        # regardless of exit status — --post reports a config's missing results
        # far better than a hung pane does.
        done_dir="$LOGDIR/.$name.done"
        rm -rf "$done_dir"; mkdir -p "$done_dir"
        for c in "${cfgs[@]}"; do
            mkbody "$LOGDIR/.$name.$c.sh" "$LOGDIR/$name.$c.log" \
                   "$base --protos $c" \
                   "$base --protos $c; rc=\$?; : > $done_dir/$c; (exit \$rc)"
        done
        mkbody "$LOGDIR/.$name.post.sh" "$log" "$base --post" \
               "while (( \$(ls $done_dir | wc -l) < ${#cfgs[@]} )); do sleep 5; done; $base --post"
        body="$LOGDIR/.$name.${cfgs[0]}.sh"
    else
        run="$cmd"
        [[ "${PIN:-0}" == 1 ]] && run="taskset -c $(( i % NCPU )) $cmd"
        body="$LOGDIR/.$name.sh"
        mkbody "$body" "$log" "$cmd" "$run"
    fi

    if (( i == 0 )); then
        tmux new-session -d -s "$SESSION" -n "$name" "$body"
    else
        tmux new-window -t "$SESSION" -n "$name" "$body"
    fi
    if (( ${#cfgs[@]} )); then
        for c in "${cfgs[@]:1}"; do
            tmux split-window -t "$SESSION:$name" "$LOGDIR/.$name.$c.sh"
            tmux select-layout -t "$SESSION:$name" tiled >/dev/null
        done
        tmux split-window -t "$SESSION:$name" "$LOGDIR/.$name.post.sh"
        tmux select-layout -t "$SESSION:$name" tiled >/dev/null
    fi
    i=$((i + 1))   # not ((i++)): that returns 1 when i was 0, which set -e traps
done

tmux select-window -t "$SESSION:^"    # first window, whatever base-index is set
echo "started $i run(s) in tmux session '$SESSION'  (logs: $LOGDIR)"
tmux list-windows -t "$SESSION" -F '  #{window_index}: #{window_name}'
[[ "${ATTACH:-1}" == 1 ]] && exec tmux attach -t "$SESSION"
echo "attach with: tmux attach -t $SESSION"
