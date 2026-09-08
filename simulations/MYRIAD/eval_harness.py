#!/usr/bin/env python3
"""Shared constants and utilities for MYRIAD eval scripts."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numbers
import os
import re
import shutil
import statistics
import subprocess
import tempfile
import sys

MYRIAD_DIR  = Path(__file__).parent
SIM_ROOT    = MYRIAD_DIR.parent
REPO        = SIM_ROOT.parent
SRC         = REPO / "src"
BIN         = str(SRC / "RoutingSim")
TOPO_DIR    = SIM_ROOT / "topologies"
TOPO_SCRIPTS = MYRIAD_DIR / "myriad_topologies"   # topo/ring generators live here
RINGLABELS_DIR = TOPO_SCRIPTS / "ringlabels"      # generated + native ring-label inis
DEFAULT_TOPO = TOPO_DIR / "grid_10,10.txt"
COMMON_INI  = str(MYRIAD_DIR / "eval_common.ini")

_PROGRESS_RE = re.compile(r"Elapsed:|% done|% complete|\bETA\b|\d+%")

# --- central run conditions --------------------------------------------------
# Everything below is read from eval_common.ini; scripts never hardcode a value
# that lives there. CLI flags (--refresh/--warmup/--snapshot/--set) land here too
# and are announced, so a printed run always matches the conditions used.
EXTRA_OPTS = []          # from --set, applied to every run
RNG_ISOLATE = True       # dynamics RNG isolated from the protocols'
PROGRESS = True          # live \r status line; off when runs go in parallel
_announced = set()


def _parse_common():
    out = {}
    for ln in Path(COMMON_INI).read_text().splitlines():
        ln = ln.split("#", 1)[0].strip()
        if "=" in ln and not ln.startswith("["):
            k, v = ln.split("=", 1)
            out[k.strip().removeprefix("**.").removeprefix("*.")] = v.strip()
    return out


COMMON = _parse_common()      # the file, for diffing
EFFECTIVE = dict(COMMON)      # the file plus CLI overrides


def common(name, default=None):
    """The value in force, i.e. eval_common.ini unless a CLI flag changed it."""
    return EFFECTIVE.get(name, default)


def _sec(v):
    v = str(v).strip()
    for suf, mul in (("ms", 1e-3), ("us", 1e-6), ("ns", 1e-9), ("s", 1.0)):
        if v.endswith(suf):
            return float(v[:-len(suf)]) * mul
    return float(v)


def refresh_s():
    return _sec(common("refreshInterval", "120s"))


def warmup_s():
    return _sec(common("collectWarmupDuration", "10s"))


def simtime(cycles=2, extra=0.0):
    """Run horizon as warmup + `cycles` refresh cycles + `extra` seconds, so
    every script's window tracks --refresh instead of pinning its own."""
    return f"{warmup_s() + cycles * refresh_s() + extra:g}s"


def overrides(**kv):
    """Per-run parameter overrides as CLI options, recorded for the announce
    line. Keys are NED parameter names ('**.' is added) or omnetpp options
    ('sim-time-limit'); values are literal ini values."""
    out = []
    for k, v in kv.items():
        if k in ("sim_time_limit", "sim-time-limit"):
            out.append(f"--sim-time-limit={v}")
        else:
            out.append(f"--**.{k}={v}")
    return out


def _changed(opts):
    """(name, common value, run value) for options that differ from eval_common."""
    diff = []
    for o in opts:
        if not o.startswith("--"):
            continue
        k, _, v = o[2:].partition("=")
        k = k.removeprefix("**.").removeprefix("*.")
        was = COMMON.get(k if k != "sim-time-limit" else "sim-time-limit")
        if was is None or was.strip('"') == v.strip('"'):
            continue
        diff.append((k, was, v))
    return diff


def announce(what, opts=(), dynamics=None, once=True):
    """Print the run conditions, what deviates from eval_common, the dynamics in
    force and the RNG setup. Called once per script section."""
    if once and what in _announced:
        return
    _announced.add(what)
    j = refresh_s() * 0.1
    print(f"  conditions: refresh {common('refreshInterval')} (jitter {j:g}s = NED 10%), "
          f"warmup {common('collectWarmupDuration')}, snapshot {common('snapshotInterval')}, "
          f"rxmt {common('rxmtInterval')}")
    diff = _changed(list(opts) + EXTRA_OPTS)
    if diff:
        print("  changed vs eval_common: "
              + ", ".join(f"{k} {was} → {now}" for k, was, now in diff))
    print(f"  dynamics: {dynamics or 'none (static topology)'}")
    print(f"  RNG: seed-set per repeat"
          + (", dynamics RNG isolated (identical failure schedule for every protocol)"
             if RNG_ISOLATE else ", SHARED with the protocols (schedules differ per protocol)"))


def dynamics_args(model, **kv):
    """CLI options + a one-line description for an active dynamics scenario.
    model: 'link' (LinkFailure), 'node' (EscalatingFailure) or 'churn'
    (NodeChurn)."""
    if model == "churn":
        desc = (f"NodeChurn — {kv.get('numNodes')} node(s) every {kv.get('period')} "
                f"from t={kv.get('warmup')}, each back after "
                f"{kv.get('downMin')}+U(0,{kv.get('downJitter')})")
    elif model == "link":
        kv.setdefault("linkSelection", '"random"')
        kv.setdefault("linkFraction", 0)     # >0 would override numLinksFail

        sel = str(kv["linkSelection"]).strip('"')  # not inline: py<3.12 bans \ in f-strings
        desc = (f"LinkFailure/{sel} — "
                f"{kv.get('numLinksFail', '?')} links, every {kv.get('period')} "
                f"for {kv.get('duration')}, "
                + ("once" if str(kv.get("oneFailure")) == "true" else "repeating"))
    else:
        waves = str(kv.get("fractions", "")).strip('"')
        desc = ("EscalatingFailure — waves of "
                + ", ".join(f"{float(f):.0%}" for f in waves.split(",") if f)
                + f" of nodes, first at t={kv.get('warmup')}, then every "
                f"{kv.get('waveSpacing')}, each node back after "
                f"{kv.get('downMin')}+U(0,{kv.get('downJitter')})")
    return [f"--**.dynamics.{k}={v}" for k, v in kv.items()], desc

# Stamp of the binary that last passed the oracle gate, so repeat eval runs on an
# unchanged build skip the (slow) regression. mtime+size is enough for a local
# build artifact — no need to hash 4.5MB every run.
_GATE_STAMP = MYRIAD_DIR / "eval" / ".oracle_gate_ok"


def _bin_sig():
    st = Path(BIN).stat()
    return f"{st.st_mtime_ns}:{st.st_size}"


def gate_up_to_date():
    """True if the oracle gate already passed for the current binary."""
    try:
        return _GATE_STAMP.read_text().strip() == _bin_sig()
    except (OSError, FileNotFoundError):
        return False


def mark_gate_passed():
    _GATE_STAMP.parent.mkdir(parents=True, exist_ok=True)
    _GATE_STAMP.write_text(_bin_sig())


def native_labels(topo_path):
    """The ring plan shipped WITH a generated topology, or None.

    ringpipe/export write the pair as csv/<sub>/<stem>.txt and
    ringlabels/<sub>/<stem>.ini, so mirror the path across the two trees. Older
    plans sit flat in ringlabels/ or beside eval_common as <stem>.native.ring.ini."""
    p = Path(topo_path).resolve()
    cand = []
    if "csv" in p.parts:
        i = len(p.parts) - 1 - p.parts[::-1].index("csv")
        cand.append(Path(*p.parts[:i], "ringlabels",
                         *p.parts[i + 1:]).with_suffix(".ini"))
    cand += [RINGLABELS_DIR / f"{p.stem}.ini",
             MYRIAD_DIR / f"{p.stem}.native.ring.ini"]
    return next((c for c in cand if c.exists()), None)


def ensure_ring_labels(topo_path, basis="tree", max_ring_size=0, echo=True):
    """Return path to <stem>.<basis>[.mN][.noecho].ring.ini,
    generating it if absent. The pruning options are part of the file name so a
    run never picks up labels built under different ones.

    A shipped native plan always wins, whatever --basis says: those topologies
    were built FROM their rings, so mining a fresh basis would measure a
    different plan than the one being evaluated."""
    import time
    p = Path(topo_path)
    native = native_labels(topo_path)
    if basis == "native" and native is None:
        sys.exit(f"--basis native but no ring labels for {p.name} "
                 f"(expected {RINGLABELS_DIR / (p.stem + '.ini')}; "
                 f"run myriad_topologies/ringpipe.py or export.py)")
    if native is not None:
        if basis != "native":
            print(f"  [ring labels] {p.name}: using shipped plan "
                  f"{native.name} (ignoring --basis {basis})", flush=True)
        check_ring_plan(topo_path, native)
        return native
    tag = (basis + (f".m{max_ring_size}" if max_ring_size else "")
           + ("" if echo else ".noecho"))
    out = RINGLABELS_DIR / f"{p.stem}.{tag}.ring.ini"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        print(f"  [ring labels] {p.name} ({tag}) …", flush=True)
        t0 = time.time()
        proc = subprocess.Popen(
            [sys.executable, str(TOPO_SCRIPTS / "topo2rings.py"),
             str(p.resolve()), "--out", str(out), "--basis", basis,
             *(["--max-ring-size", str(max_ring_size)] if max_ring_size else []),
             *([] if echo else ["--no-echo"])],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in proc.stdout:
            sys.stderr.write(f"\r    {line.rstrip()[:100]:<100}")
            sys.stderr.flush()
        sys.stderr.write("\r" + " " * 104 + "\r")
        sys.stderr.flush()
        proc.wait()
        if proc.returncode != 0 or not out.exists():
            sys.exit(f"topo2rings.py failed for {p.name}")
        print(f"  [ring labels] generated {out.name} ({time.time()-t0:.0f}s)", flush=True)
    check_ring_plan(topo_path, out)
    return out


_RING_CHECKED = set()


def check_ring_plan(topo_path, labels_path):
    """Warn if the plan isn't walkable by GP; see myriad_topologies/ringcheck.py.

    Every eval routes through ensure_ring_labels, so checking here covers native
    and generated plans alike. A warning, not an exit: an unwalkable ring makes
    that ring's laps circle until the hop cap kills them, which skews overhead
    and reconvergence but still produces a run worth looking at."""
    key = (str(topo_path), str(labels_path))
    if key in _RING_CHECKED:
        return {}
    _RING_CHECKED.add(key)
    sys.path.insert(0, str(TOPO_SCRIPTS))
    import ringcheck
    bad = ringcheck.check(topo_path, labels_path)
    if bad:
        worst = sorted(bad.items())[:3]
        print(f"  ! {Path(topo_path).name}: {len(bad)} of "
              f"{len(ringcheck.load(topo_path, labels_path)[0])} rings are not "
              f"walkable (a member has !=2 ring-neighbours, so GP's lap can fork "
              f"off the cycle):", flush=True)
        for r, off in worst:
            print(f"      ring {r}: {off}", flush=True)
        if len(bad) > len(worst):
            print(f"      … and {len(bad) - len(worst)} more; list them with "
                  f"myriad_topologies/ringcheck.py {topo_path} {labels_path}",
                  flush=True)
    return bad


def ring_label_lines(labels_path):
    """Read a .ring.ini and return non-comment content lines."""
    with open(labels_path) as f:
        return [ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")]


# Crash lines too: a SIGABRT prints its reason through libstdc++/glibc, not
# through OMNeT++'s error channel, so none of the words below appear in it.
_ERR_RE = re.compile(r"<!>|Error|error|Fatal|Exception|ASSERT|budget expired"
                     r"|ring broken|terminate called|bad_alloc|what\(\)"
                     r"|Assertion|double free|corruption|Segmentation")


def _fail(msg, cmd, log):
    """Exit naming the run and quoting the error, not the log tail.

    Under --jobs several sims share the terminal, so a bare FAIL says nothing
    about which one died; and OMNeT++ dumps its undisposed objects AFTER the
    error, so the last lines are reliably the least useful ones. The whole log
    and the exact command go to a file either way: a sim that dies without
    printing anything (an abort takes the unflushed stdout with it) leaves
    nothing to quote, and then the only way on is to rerun it by hand."""
    import shlex
    args = [str(a) for a in cmd]
    ini = next((a for a in reversed(args) if a.endswith(".ini")), "?")
    cfg = next((b for a, b in zip(args, args[1:]) if a == "-c"), "")
    errs = [ln.rstrip() for ln in log.splitlines() if _ERR_RE.search(ln)]
    line = f"cd {SIM_ROOT} && " + shlex.join(args)
    dump = MYRIAD_DIR / "eval" / "logs" / f"fail_{Path(ini).stem}_{os.getpid()}.log"
    try:
        dump.parent.mkdir(parents=True, exist_ok=True)
        dump.write_text(f"# {msg}\n# {line}\n\n{log}")
        where = f"\n  full log: {dump}"
    except OSError as e:
        where = f"\n  (could not write the log: {e})"
    sys.exit(f"FAIL: {msg}\n  ini: {os.path.basename(ini)}"
             f"{' -c ' + cfg if cfg else ''}\n  "
             + "\n  ".join(errs[-12:] or log.splitlines()[-12:] or
                           ["(the sim printed nothing)"])
             + f"{where}\n  rerun: {line}")


def _exit_reason(rc):
    """What a nonzero return code means, in words. A negative one is a signal:
    the sim was killed rather than exiting, and which signal narrows it a lot."""
    if rc >= 0:
        return f"sim exited {rc}"
    import signal
    try:
        sig = signal.Signals(-rc)
    except ValueError:
        return f"sim killed by signal {-rc}"
    hint = {"SIGABRT": "uncaught C++ exception or failed assert — commonly "
                       "std::bad_alloc, i.e. out of memory: lower --jobs",
            "SIGKILL": "killed from outside — usually the OOM killer: "
                       "lower --jobs",
            "SIGSEGV": "segfault in the model",
            "SIGINT": "interrupted"}.get(sig.name, "")
    return f"sim killed by {sig.name}" + (f" ({hint})" if hint else "")


def run_sim(cmd, cwd=None, allow_budget_expired=False, rep=0):
    """Run simulator with live progress; exit on nonzero return or fatal patterns.

    rep: repeat index. Its seed-set replaces any the caller passed, so repeats
    differ in RNG draws while rep 0 reproduces the scripts' historic --seed-set=1.
    Only rep 0 streams progress — parallel repeats would fight over the line."""
    full_cmd = [a for a in cmd if not str(a).startswith(("--seed-set", "--num-rngs"))]
    full_cmd += EXTRA_OPTS + [f"--seed-set={rep + 1}", "--cmdenv-status-frequency=2s"]
    if RNG_ISOLATE:
        full_cmd += ["--num-rngs=2", "--**.dynamics.rng-0=1"]
    # Line-buffer the child: writing to a pipe, its stdout is block-buffered, so
    # a crash (an abort, a signal) discards whatever error message is still in
    # that buffer and the run looks like it died silently.
    launch = (["stdbuf", "-oL", "-eL"] if shutil.which("stdbuf") else []) + full_cmd
    proc = subprocess.Popen(launch, cwd=str(cwd or SIM_ROOT),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    lines = []
    for line in proc.stdout:
        lines.append(line)
        if PROGRESS and rep == 0 and _PROGRESS_RE.search(line):
            sys.stderr.write(f"\r    {line.rstrip()[:100]:<100}")
            sys.stderr.flush()
    if PROGRESS and rep == 0:
        sys.stderr.write("\r" + " " * 104 + "\r")
        sys.stderr.flush()
    proc.wait()
    log = "".join(lines)
    if proc.returncode != 0:
        _fail(_exit_reason(proc.returncode), full_cmd, log)
    if not allow_budget_expired and ("budget expired" in log or "ring broken" in log):
        grace = (warmup_s() + _sec(common("deadInterval", "40s"))
                 + _sec(common("helloInterval", "10s")))
        stamps = [float(m.group(1)) for m in
                  re.finditer(r"\[t=([0-9.]+)[^\]]*\][^\n]*(?:budget expired|ring broken)", log)]
        if not stamps or any(t > grace for t in stamps):
            _fail("budget expired or ring broken", full_cmd, log)
    return proc


# Everything an eval names -- config keys, result dirs, CSV values, plot text --
# calls the path-vector protocol `pv`. The NED module keeps its original name, so
# this map is the one place the two vocabularies meet.
NED_MODULE = {"pv": "gp", "ls": "ls"}

# One colour per protocol variant, everywhere. A reader who learns "red is
# path-vector" on one figure must not have to relearn it on the next, so this is
# the only place a series colour is chosen. A second metric for the SAME variant
# in one figure keeps the colour and changes the marker/linestyle/alpha instead.
PROTO_COLOR = {"pv": "tab:red", "ls": "tab:blue",
               "pv_noreplay": "darkturquoise", "pv_nospof": "tab:purple"}
# Figure- and table-facing names, keyed the same way (see eval_dynamics.CONFIGS).
PROTO_LABEL = {"pv": "Path-Vector (PV)", "ls": "Link-State (LS)",
               "pv_noreplay": "Path-Vector (refresh-bound)",
               "pv_nospof": "Path-Vector (spof-blind)"}


def write_proto_ini(out_path, proto, topo_path, ring_labels_path,
                    extra_lines=None, update_jitter=None, warmup=None,
                    network="routingsim.simulations.DefaultNetwork"):
    """Self-contained ini for one protocol+topology: eval_common plus the per-run
    bits. myriad{gp,ls}.ini is deliberately not loaded — its [Config] sections
    would override eval_common."""
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{NED_MODULE[proto]}"',
        f"network = {network}",
        f'*.manager.topologyFile = "{Path(topo_path).resolve()}"',
        "*.mainStatistics.samplingInterval = 0.01s",
    ]
    for par, val in (("updateJitter", update_jitter), ("collectWarmupDuration", warmup)):
        if val is None:
            continue
        v = str(val)
        if v[-1].isdigit():
            v += "s"
        lines.append(f"**.{par} = {v}")
    if extra_lines:
        lines.extend(extra_lines)
    lines.extend(ring_label_lines(ring_labels_path))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=Path(out_path).parent,
                               prefix=Path(out_path).name + ".", suffix=".tmp")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, out_path)
    return str(out_path)


# --- repeated runs -----------------------------------------------------------
# Set from --repeat/--jobs by eval_entrypoint (or a script's own arg parsing).
REPEATS = 1
JOBS = None


class Stat(float):
    """Mean of a measurement across repeats, carrying its stdev in .sd.

    A float subclass so every existing format spec, arithmetic and plot call in
    the eval scripts keeps working unchanged; only summary tables opt into ±."""
    def __new__(cls, mean, sd=0.0):
        v = super().__new__(cls, mean)
        v.sd = sd
        return v


def pm(v, spec=".1f"):
    """'±sd' suffix for a Stat, or '' for a plain single-run value."""
    sd = getattr(v, "sd", None)
    return f"±{sd:{spec}}" if sd is not None else ""


_BYTE_UNITS = ((1e9, "GB"), (1e6, "MB"), (1e3, "kB"), (1.0, "B"))


def byte_unit(vals):
    """(divisor, name) for a whole COLUMN or plot axis, chosen from its largest
    value. One unit for the set, not per value: a column mixing '4.1 kB' and
    '2.3 MB' cannot be scanned or ranked by eye, and an axis has one scale by
    definition. Use fmt_bytes for a standalone value instead."""
    m = max((abs(v) for v in vals if v is not None), default=0.0)
    for div, name in _BYTE_UNITS:
        if m >= div:
            return div, name
    return 1.0, "B"


def fmt_bytes(v, unit=None):
    """One value at its own scale, for a log line: '812 B', '4.07 kB', '1.53 GB'.
    Three significant figures, so the width stays put as the magnitude moves.
    Pass `unit` (a byte_unit() pair) to hold a column to a single scale."""
    if v is None:
        return "-"
    div, name = unit or byte_unit([v])
    q = v / div
    return f"{q:,.0f} {name}" if div == 1.0 else f"{q:,.3g} {name}"


def repeats_note():
    """Table-header suffix stating the repeat count (empty for a single run)."""
    return f" [mean±sd over {REPEATS} runs]" if REPEATS > 1 else ""


def rep_path(path, rep):
    """Per-repeat result dir / dump file so parallel runs never share one."""
    return path if rep == 0 else f"{path}.r{rep}"


def _agg(vals):
    """Recursively average repeat results: numbers -> Stat(mean, stdev), dicts
    over their common keys, sequences elementwise; anything else -> rep 0."""
    first = vals[0]
    if isinstance(first, bool) or not isinstance(first, (numbers.Number, dict, list, tuple)):
        return first
    if isinstance(first, numbers.Number):
        return Stat(statistics.fmean(vals),
                    statistics.pstdev(vals) if len(vals) > 1 else 0.0)
    if isinstance(first, dict):
        keys = set(first)
        for v in vals[1:]:
            keys &= set(v)
        return {k: _agg([v[k] for v in vals]) for k in first if k in keys}
    n = min(len(v) for v in vals)
    out = [_agg([v[i] for v in vals]) for i in range(n)]
    return tuple(out) if isinstance(first, tuple) else out


def repeat_mean(fn, repeats=None):
    """Run fn(rep) REPEATS times in parallel and average the returned structure.

    fn must route its result dir / dump paths through rep_path() and pass rep=
    into run_sim(). At repeats==1 the raw result is passed through untouched."""
    n = repeats or REPEATS
    if n <= 1:
        return fn(0)
    with ThreadPoolExecutor(max_workers=JOBS or min(n, os.cpu_count() or 1)) as ex:
        return _agg(list(ex.map(fn, range(n))))


def add_repeat_args(parser):
    parser.add_argument("--repeat", type=int, default=1, metavar="N",
                        help="run every measurement N times and report mean±sd")
    parser.add_argument("--jobs", type=int, default=None, metavar="J",
                        help="parallel sims (default: min(--repeat, cpu count))")
    parser.add_argument("--refresh", default=None, metavar="T",
                        help=f"refreshInterval for every run (eval_common: "
                             f"{common('refreshInterval')}); jitter follows at 10%%")
    parser.add_argument("--snapshot", default=None, metavar="T",
                        help=f"snapshotInterval = latency resolution (eval_common: "
                             f"{common('snapshotInterval')})")
    parser.add_argument("--set", dest="set_opts", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="override any NED parameter on every run, repeatable "
                             "(e.g. --set holdInterval=0s)")
    parser.add_argument("--no-rng-isolate", dest="rng_isolate", action="store_false",
                        help="let dynamics draw from the shared RNG; protocols then "
                             "see different failure schedules (default: isolated)")


_PICK = {"first": 0, "lo": 0, "min": 0, "last": -1, "hi": -1, "max": -1,
         "median": None, "mid": None, "med": None}   # None = middle by length


def resolve_topo_pick(spec):
    """'<dir>@first|median|last' -> one *.txt from that dir's transition sweep,
    sorted by _tr<N> (then name) and picked by position. Lets eval_runs.txt name
    a representative of a swept family without pinning a brittle tr/fan/seed
    filename that regeneration renames. A plain path (no '@<pick>' suffix) is
    returned unchanged."""
    d, sep, sel = spec.rpartition("@")
    if not sep or sel not in _PICK:
        return spec
    tr = lambda f: (int(m.group(1)) if (m := re.search(r"_tr(\d+)", f.name)) else 0)
    files = sorted(Path(d).glob("*.txt"), key=lambda f: (tr(f), f.name))
    if not files:
        sys.exit(f"--topo {spec}: no *.txt in {d}")
    i = _PICK[sel]
    return str(files[len(files) // 2 if i is None else i])


def apply_repeat_args(args):
    """Bind the shared CLI flags into the module state run_sim reads."""
    global REPEATS, JOBS, EXTRA_OPTS, RNG_ISOLATE, TOPO_LABEL
    if getattr(args, "topo", None):
        args.topo = resolve_topo_pick(args.topo)
        TOPO_LABEL = topo_label(args.topo)
    REPEATS = max(1, getattr(args, "repeat", 1) or 1)
    JOBS = getattr(args, "jobs", None)
    RNG_ISOLATE = getattr(args, "rng_isolate", True)
    EXTRA_OPTS = [f"--**.{s}" if not s.startswith("-") else s
                  for s in getattr(args, "set_opts", [])]
    for s in getattr(args, "set_opts", []):        # --set must move EFFECTIVE too
        k, _, v = s.lstrip("-").partition("=")
        if v:
            EFFECTIVE[k.rsplit(".", 1)[-1]] = v
    for flag, par in (("refresh", "refreshInterval"), ("snapshot", "snapshotInterval"),
                      ("warmup", "collectWarmupDuration")):
        v = getattr(args, flag, None)
        if v:
            if str(v)[-1].isdigit():
                v = f"{v}s"
            EXTRA_OPTS.append(f"--**.{par}={v}")
            EFFECTIVE[par] = str(v)


TOPO_LABEL = ""          # set by apply_repeat_args, for plot titles


def topo_label(path):
    """Short plot-title name for a topology file: 'rgg_n750_d4.5_seed0.txt' ->
    'RGG 750', 'grid_10,10.txt' -> 'GRID 10,10'. Family tokens uppercase, the
    size kept, generator parameters (seed/d/m/p/r/k/cs) dropped."""
    fam, size = [], []
    for t in Path(path).stem.split("_"):
        if re.fullmatch(r"n?\d[\d.,]*", t):           # n750, 10,10, 100
            size.append(t.lstrip("n"))
        elif re.fullmatch(r"(seed|d|m|p|r|k|cs)[\d.]+", t):
            continue                                  # generator knob, not identity
        elif not size:                                # family words precede the size
            fam.append(t.upper())
    return " ".join(fam + size)


def title(base):
    """Plot title with the topology on its own line, when a script bound one.

    Second line, not appended: a generated topology's name carries its whole
    parameter set (RINGGROW DELAUNAY A300 SPOF0.5 RL10 TR500 ...), which runs off
    both ends of the figure when it shares the line with the description."""
    return f"{base}\n{TOPO_LABEL}" if TOPO_LABEL else base


def add_topo_args(parser):
    add_repeat_args(parser)
    parser.add_argument(
        "--topo", default=str(DEFAULT_TOPO),
        help=f"topology file (default: {DEFAULT_TOPO.name})")
    parser.add_argument(
        "--basis", choices=["min", "tree", "all", "native"], default="tree",
        help="ring basis: tree=cycle_basis (spanning-tree, default), "
             "min=minimum_cycle_basis; 'all' aliases tree; "
             "native=use a hand-provided <stem>.native.ring.ini as-is "
             "(for topologies built FROM a ring plan, e.g. myriad_topologies/ringpipe.py)")
    parser.add_argument(
        "--max-ring-size", dest="max_ring_size", type=int, default=0,
        help="drop rings larger than this; members fall back to echo rings "
             "(0 = keep all). Long rings dominate GP's fan-out and lap latency")
    parser.add_argument(
        "--no-echo-rings", dest="no_echo_rings", action="store_true",
        help="omit 2-node echo rings; combined with --max-ring-size this can "
             "leave nodes in no ring at all, which the label step reports")
    parser.add_argument(
        "--update-jitter", dest="update_jitter", default=None,
        help="GP initial-UPDATE jitter; raise on large topologies, e.g. 15s")
    parser.add_argument(
        "--warmup", dest="warmup", default=None,
        help=f"collectWarmupDuration (eval_common: {common('collectWarmupDuration')})")
    parser.add_argument(
        "--tag", default=None,
        help="suffix appended to the output dir, to keep several runs on the "
             "same topology (different --set/--basis/…) from overwriting each "
             "other, e.g. --tag noflood")
