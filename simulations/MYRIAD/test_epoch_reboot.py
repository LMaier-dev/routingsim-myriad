#!/usr/bin/env python3
"""Generation numbers must never go backwards, including across a reboot.

reset() wipes memberships and the RIB, so originSeq used to restart at 1 while
every other router still held that ring at its pre-reboot generation. store()
drops anything below curSeq (MyriadGpRib.cc), so the rebooted king's genuine
updates were discarded as stale until its counter climbed back past where it
already was. MyriadGP::lastOriginSeq is the stable-storage floor that prevents
this; this test is what fails if it is removed.

    python3 simulations/MYRIAD/test_epoch_reboot.py

Exit 0 = no (node, ring) ever re-used or lowered a generation number.

Non-vacuity is checked too: the run must actually have failed nodes, and some
router must have originated both before and after the first wave. It does not
prove the originating router is one that rebooted, so treat a pass as "the
invariant held over a run with reboots in it", not as a targeted reproduction.
"""
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
SRC = os.path.join(ROOT, "src")
TOPO = os.path.join(SIM, "MYRIAD", "myriad_topologies", "csv",
                    "myriad_paperexample.txt")
WARMUP = 45.0       # first failure; also the straddle point for non-vacuity

INI = f"""[General]
ned-path={SRC};{SIM}
**.protocols = "routingsim.routing.MYRIAD.myriadgp"
network = routingsim.simulations.MYRIAD.ChurnNetwork
include {os.path.join(SIM, "MYRIAD", "eval_common.ini")}

[Config EpochReboot]
*.manager.topologyFile = "{TOPO}"
*.mainStatistics.samplingInterval = 0.01s
sim-time-limit = 300s
cmdenv-express-mode = false
cmdenv-event-banners = false
**.logLevel = "info"
**.colorLog = false
# Sustained churn, not waves. A king that reboots comes back as a plain member,
# because collectWins lets a live member beat a crashed one, so a single wave
# never makes it re-originate. It reclaims the crown on a later election (ties
# go to the higher id), and THAT is when a restarted counter shows up. Churning
# one node every couple of seconds gets there; waves do not.
*.dynamics.numNodes = 1
*.dynamics.period = 10s
*.dynamics.warmup = {WARMUP}s
*.dynamics.downMin = 2s
*.dynamics.downJitter = 2s
# Rings must be whole again between failures, or laps never close, no election
# ever completes and nothing originates at all. On seven nodes, one router down
# every 2s is enough to stop the protocol dead.
**.refreshInterval = 30s
**.Node0.*.ringLabel = "1000000"
**.Node1.*.ringLabel = "1000000,999999"
**.Node2.*.ringLabel = "999999,999998"
**.Node3.*.ringLabel = "1000000,999999"
**.Node4.*.ringLabel = "999999"
**.Node5.*.ringLabel = "999998"
**.Node6.*.ringLabel = "999999,999998"
"""

ORIGINATE = re.compile(
    r"\[t=([0-9.eE+-]+) n=(\d+).*?originate UPDATE ring='(\d+)' seq=(\d+)")
# NodeChurn logs nothing of its own, so the evidence that routers actually went
# away is the protocol reacting to it: onNeighborDead probes the dead neighbour.
DEATH = re.compile(r"to dead neighbor")


def binary():
    """Release first, matching the eval scripts. It prints which one it picked
    and how old it is on purpose: building only MODE=debug leaves a stale
    release binary in place, this picks it up silently, and the run then scores
    code that is not the code just edited."""
    import time
    for name in ("RoutingSim", "RoutingSim_dbg"):
        p = os.path.join(SRC, name)
        if os.path.exists(p):
            age = time.time() - os.path.getmtime(p)
            print(f"using {name} (built {age / 60:.0f} min ago)")
            return p
    sys.exit(f"no RoutingSim binary in {SRC} — build it first")


def run():
    with tempfile.NamedTemporaryFile("w", suffix=".ini", delete=False) as f:
        f.write(INI)
        ini = f.name
    try:
        r = subprocess.run([binary(), "-u", "Cmdenv", "-c", "EpochReboot", ini],
                           cwd=SRC, capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"simulation failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
        return r.stdout
    finally:
        os.remove(ini)


def main():
    out = run()
    failed = len(DEATH.findall(out))
    seen = {}          # (node, ring) -> last seq
    straddlers = set()  # (node, ring) that originated on both sides of WARMUP
    before, after = set(), set()
    bad = []
    for t, node, ring, seq in ORIGINATE.findall(out):
        key, t, seq = (int(node), int(ring)), float(t), int(seq)
        (before if t < WARMUP else after).add(key)
        if key in seen and seq <= seen[key]:
            bad.append(f"n={key[0]} ring={key[1]}: seq {seen[key]} -> {seq} at t={t:g}")
        seen[key] = seq
    straddlers = before & after

    if not seen:
        sys.exit("FAIL: no 'originate UPDATE' lines parsed — log format changed?")
    if failed == 0:
        sys.exit("FAIL: no nodes were failed, the run never exercised a reboot")
    if not straddlers:
        sys.exit("FAIL: nobody originated on both sides of the first wave")
    for m in bad:
        print("FAIL", m, file=sys.stderr)
    if bad:
        sys.exit(f"{len(bad)} generation regression(s): a reboot re-used a seq")
    print(f"ok  {len(seen)} (node,ring) generations, {failed} node failures, "
          f"{len(straddlers)} straddling the first wave — no seq ever repeated "
          "or decreased")


if __name__ == "__main__":
    main()
