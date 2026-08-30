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

mapfile -t RUNS < <(grep -vE '^\s*(#|$)' "$RUNFILE")
(( ${#RUNS[@]} )) || { echo "no runs in $RUNFILE" >&2; exit 1; }
(( ${#RUNS[@]} <= NCPU )) || echo "note: ${#RUNS[@]} runs on $NCPU cores — they will share" >&2

i=0
for cmd in "${RUNS[@]}"; do
    # Window name from the distinguishing options, so tabs are tellable apart:
    # script, topology stem, and the flags that actually vary between runs.
    # `|| true` on each: grep exits 1 when a run has no --topo/--only, and
    # set -e would abort the whole script on that.
    script=$(sed -E 's#.*eval_([a-z_]+)\.py.*#\1#' <<<"$cmd")
    topo=$(grep -oE '\-\-topo +\S+' <<<"$cmd" | sed -E 's#.*/##; s/\.txt//') || true
    opts=$(grep -oE '\-\-(only|basis) +\S+' <<<"$cmd" | awk '{print $2}' | paste -sd_ -) || true
    slug=$(sed -E 's/[^a-zA-Z0-9]+/_/g; s/^_|_$//g' \
           <<<"${script}_${topo}_${opts}" | cut -c1-32)
    name="$(printf '%02d' $i)_${slug}"
    log="$LOGDIR/$name.log"

    run="$cmd"
    if [[ "${PIN:-0}" == 1 ]]; then
        run="taskset -c $(( i % NCPU )) $cmd"
    fi
    # The run goes into a temp script rather than being interpolated into the
    # tmux command string: quoting survives one fewer shell that way, and a run
    # line with quotes/globs reaches python exactly as written in the runfile.
    # `exec bash` keeps the pane alive after the run so its tail is readable.
    body_sh="$LOGDIR/.$name.sh"
    { printf '#!/usr/bin/env bash\ncd %q || exit 1\n' "$WORKDIR"
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
