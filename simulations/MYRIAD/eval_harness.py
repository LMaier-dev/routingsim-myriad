#!/usr/bin/env python3
"""Shared constants and utilities for MYRIAD eval scripts."""
from pathlib import Path
import re
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


def run_sim(cmd, cwd=None, allow_budget_expired=False):
    """Run simulator with live progress; exit on nonzero return or fatal patterns."""
    full_cmd = list(cmd) + ["--cmdenv-status-frequency=2s"]
    proc = subprocess.Popen(full_cmd, cwd=str(cwd or SIM_ROOT),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    lines = []
    for line in proc.stdout:
        lines.append(line)
        if _PROGRESS_RE.search(line):
            sys.stderr.write(f"\r    {line.rstrip()[:100]:<100}")
            sys.stderr.flush()
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
    """Write a self-contained ini for one protocol+topology.

    Includes eval_common.ini (fair-timing knobs) then adds topology + ring
    labels. Avoids loading myriad{gp,ls}.ini so their [Config] overrides
    (e.g. refreshInterval=15s vs eval_common's 100s) don't fire.

    update_jitter: override MyriadGp's initial-UPDATE jitter (NED default 1s).
    Pass a larger value for big topologies where 1s isn't enough to spread the
    startup storm. Accepts "10s" or bare "10" (seconds assumed). Ignored by LS.

    warmup: override MyriadGp's collectWarmupDuration (NED default 10s). Same
    unit handling as update_jitter. Ignored by LS.
    """
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
        f"network = {network}",
        f'*.manager.topologyFile = "{Path(topo_path).resolve()}"',
        "**.mobility = false",
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


def add_topo_args(parser):
    """Add --topo, --basis, --update-jitter to an ArgumentParser."""
    parser.add_argument(
        "--topo", default=str(DEFAULT_TOPO),
        help=f"topology file (default: {DEFAULT_TOPO.name})")
    parser.add_argument(
        "--basis", choices=["min", "all"], default="all",
        help="ring basis: all=cycle_basis (default), min=minimum_cycle_basis")
    parser.add_argument(
        "--update-jitter", dest="update_jitter", default=None,
        help="override GP initial-UPDATE jitter (default 1s); raise for large "
             "topologies to spread the startup storm, e.g. --update-jitter 15s")
    parser.add_argument(
        "--warmup", dest="warmup", default=None,
        help="override GP collectWarmupDuration (default 10s); raise for large "
             "topologies where counts need longer to settle, e.g. --warmup 30s")
