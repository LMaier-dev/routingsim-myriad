#!/usr/bin/env python3
"""Shared constants and utilities for MYRIAD eval scripts."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numbers
import os
import re
import statistics
import subprocess
import sys

MYRIAD_DIR  = Path(__file__).parent
SIM_ROOT    = MYRIAD_DIR.parent
REPO        = SIM_ROOT.parent
SRC         = REPO / "src"
BIN         = str(SRC / "RoutingSim")
TOPO_DIR    = SIM_ROOT / "topologies"
DEFAULT_TOPO = TOPO_DIR / "grid_10,10.txt"
COMMON_INI  = str(MYRIAD_DIR / "eval_common.ini")

_PROGRESS_RE = re.compile(r"Elapsed:|% done|% complete|\bETA\b|\d+%")

# --- central run conditions --------------------------------------------------
# Everything below is read from eval_common.ini; scripts never hardcode a value
# that lives there. CLI flags (--refresh/--warmup/--snapshot/--set) land here too
# and are announced, so a printed run always matches the conditions used.
EXTRA_OPTS = []          # from --set, applied to every run
RNG_ISOLATE = True       # dynamics RNG isolated from the protocols'
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
    model: 'link' (LinkFailure) or 'node' (EscalatingFailure)."""
    if model == "link":
        kv.setdefault("linkSelection", '"random"')
        kv.setdefault("linkFraction", 0)     # >0 would override numLinksFail

        desc = (f"LinkFailure/{str(kv['linkSelection']).strip('\"')} — "
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


def ensure_ring_labels(topo_path, basis="all"):
    """Return path to <stem>.<basis>.ringlabels.ini, generating it if absent."""
    import time
    p = Path(topo_path)
    out = MYRIAD_DIR / f"{p.stem}.{basis}.ringlabels.ini"
    if not out.exists():
        print(f"  [ring labels] {p.name} ({basis}) …", flush=True)
        t0 = time.time()
        proc = subprocess.Popen(
            [sys.executable, str(MYRIAD_DIR / "topo2rings.py"),
             str(p), "--out", str(out), "--no-png"],
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
    return out


def ring_label_lines(labels_path):
    """Read a .ringlabels.ini and return non-comment content lines."""
    with open(labels_path) as f:
        return [ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")]


def run_sim(cmd, cwd=None, allow_budget_expired=False, rep=0):
    """Run simulator with live progress; exit on nonzero return or fatal patterns.

    rep: repeat index. Its seed-set replaces any the caller passed, so repeats
    differ in RNG draws while rep 0 reproduces the scripts' historic --seed-set=1.
    Only rep 0 streams progress — parallel repeats would fight over the line."""
    full_cmd = [a for a in cmd if not str(a).startswith(("--seed-set", "--num-rngs"))]
    full_cmd += EXTRA_OPTS + [f"--seed-set={rep + 1}", "--cmdenv-status-frequency=2s"]
    if RNG_ISOLATE:
        full_cmd += ["--num-rngs=2", "--**.dynamics.rng-0=1"]
    proc = subprocess.Popen(full_cmd, cwd=str(cwd or SIM_ROOT),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    lines = []
    for line in proc.stdout:
        lines.append(line)
        if rep == 0 and _PROGRESS_RE.search(line):
            sys.stderr.write(f"\r    {line.rstrip()[:100]:<100}")
            sys.stderr.flush()
    if rep == 0:
        sys.stderr.write("\r" + " " * 104 + "\r")
        sys.stderr.flush()
    proc.wait()
    log = "".join(lines)
    if proc.returncode != 0:
        sys.exit(f"FAIL: sim exited {proc.returncode}\n{log[-1500:]}")
    if not allow_budget_expired and ("budget expired" in log or "ring broken" in log):
        sys.exit(f"FAIL: budget expired or ring broken\n{log[-1500:]}")
    return proc


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
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
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
    Path(out_path).write_text("\n".join(lines) + "\n")
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


def apply_repeat_args(args):
    """Bind the shared CLI flags into the module state run_sim reads."""
    global REPEATS, JOBS, EXTRA_OPTS, RNG_ISOLATE
    REPEATS = max(1, getattr(args, "repeat", 1) or 1)
    JOBS = getattr(args, "jobs", None)
    RNG_ISOLATE = getattr(args, "rng_isolate", True)
    EXTRA_OPTS = [f"--**.{s}" if not s.startswith("-") else s
                  for s in getattr(args, "set_opts", [])]
    for flag, par in (("refresh", "refreshInterval"), ("snapshot", "snapshotInterval"),
                      ("warmup", "collectWarmupDuration")):
        v = getattr(args, flag, None)
        if v:
            if str(v)[-1].isdigit():
                v = f"{v}s"
            EXTRA_OPTS.append(f"--**.{par}={v}")
            EFFECTIVE[par] = str(v)


def add_topo_args(parser):
    add_repeat_args(parser)
    parser.add_argument(
        "--topo", default=str(DEFAULT_TOPO),
        help=f"topology file (default: {DEFAULT_TOPO.name})")
    parser.add_argument(
        "--basis", choices=["min", "all"], default="all",
        help="ring basis: all=cycle_basis (default), min=minimum_cycle_basis")
    parser.add_argument(
        "--update-jitter", dest="update_jitter", default=None,
        help="GP initial-UPDATE jitter; raise on large topologies, e.g. 15s")
    parser.add_argument(
        "--warmup", dest="warmup", default=None,
        help=f"collectWarmupDuration (eval_common: {common('collectWarmupDuration')})")
