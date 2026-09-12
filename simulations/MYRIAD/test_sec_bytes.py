#!/usr/bin/env python3
"""Sanity gate for the secbytes_* accounting (see src/routing/MYRIAD/SECURITY.md).

The counters are pure bookkeeping — nothing is sent — so nothing else in the sim
notices if they silently stop counting. The failure that actually happens is a
dispatch one in gpSecBytes(): a PDU class falls through every branch and
contributes zero. These bounds catch that.

    python3 simulations/MYRIAD/test_sec_bytes.py <result-dir> [...]

Exit 0 = the accounting is self-consistent.
"""
import csv
import glob
import os
import shutil
import subprocess
import sys
from collections import Counter

SIG = 48        # SEC_SIG in MyriadGp.cc — keep in step
LINK_MAC = 16   # SEC_LINK_MAC: every PDU except Hello
LINK_SIG = 48   # SEC_LINK_SIG: Hello is signed, it bootstraps the MAC key
HELLO_X = 10    # SEC_HELLO: nonce + counter carried on every Hello


def scalars(d):
    s = Counter()
    for f in glob.glob(os.path.join(d, "*.sca")):
        for line in open(f):
            if line.startswith("scalar "):
                p = line.split()
                if len(p) >= 4:
                    try:
                        s[p[2]] += float(p[3])
                    except ValueError:
                        pass
    return s


def _vector_total(d, name="txBytesSec"):
    """Sum one vector across every node, or None if it cannot be read."""
    vecs = glob.glob(os.path.join(d, "*.vec"))
    tool = shutil.which("opp_scavetool")
    if not vecs or not tool:
        return None
    out = os.path.join(d, "_sec_gate_export.csv")
    try:
        subprocess.run([tool, "export", "-f", f"name =~ {name}", "-F", "CSV-R",
                        "-o", out, vecs[0]], check=True, capture_output=True, text=True)
        csv.field_size_limit(sys.maxsize)
        total, count, seen = 0.0, 0, False
        with open(out, newline="") as f:
            for row in csv.DictReader(f):
                if row["type"] == "vector" and row.get("name") == name:
                    seen = True
                    vals = row["vecvalue"].split()
                    count += len(vals)
                    total += sum(float(v) for v in vals)
        # A result dir written before the vector existed has no rows at all.
        # That is "cannot check", not "disagrees by 100%".
        return (total, count) if seen else None
    except (subprocess.CalledProcessError, OSError, ValueError):
        return None
    finally:
        if os.path.exists(out):
            os.remove(out)


def _check_vector_agrees(d, s):
    """Per-send vector vs the finish()-time scalars, by count and by total.

    The COUNT is the sharper of the two. Link authentication is charged on every
    PDU, so there must be exactly one sample per send; a vector written only
    where GP PDUs are counted silently drops Hello, and Hello is a rounding
    error in a churn run (~0.05% of bytes) but is ~100% of an idle window. The
    byte comparison would wave that through; the count cannot."""
    got = _vector_total(d)
    if got is None:
        return []
    vec, count = got
    bad = []
    if s["sent_total"] and count != int(s["sent_total"]):
        bad.append(f"txBytesSec has {count:,} samples but {int(s['sent_total']):,}"
                   " PDUs were sent — some PDU type never records one")
    scal = s["secbytes_sig"] + s["secbytes_struct"] + s["secbytes_link"]
    if scal and abs(vec - scal) / scal > 0.001:
        bad.append(f"txBytesSec sums to {vec:,.0f} but the scalars say {scal:,.0f}"
                   f" ({100 * (vec - scal) / scal:+.1f}%) — the two accounting"
                   " paths have drifted apart")
    return bad


def check(d):
    s = scalars(d)
    if not s:
        return [f"{d}: no .sca files"]
    if not s["sent_total"]:
        return [f"{d}: no sends recorded"]
    bad = []
    # Link authentication is charged once per sent PDU, Hello included, but not
    # at one rate: Hello is signed because it bootstraps the key the MAC needs.
    want_link = (s["sent_hello"] * (LINK_SIG + HELLO_X)
                 + (s["sent_total"] - s["sent_hello"]) * LINK_MAC)
    if abs(s["secbytes_link"] - want_link) > 0.5:
        bad.append(f"secbytes_link {s['secbytes_link']:.0f} != "
                   f"hello*{LINK_SIG + HELLO_X} + rest*{LINK_MAC}"
                   f" ({want_link:.0f})")
    # Signatures come in whole 48 B units.
    if s["secbytes_sig"] % SIG:
        bad.append(f"secbytes_sig {s['secbytes_sig']:.0f} not a multiple of {SIG}")
    # Every UPDATE and DEGRADE carries at least the origin/culprit signature,
    # and every transmitted batch carries at least one entry with a non-empty
    # chain, so two signatures.
    floor = SIG * (s["sent_update"] + s["sent_degrade"]) + 2 * SIG * s["sent_update_batch"]
    if s["secbytes_sig"] < floor:
        bad.append(f"secbytes_sig {s['secbytes_sig']:.0f} below floor {floor:.0f}"
                   " — a PDU class is falling through gpSecBytes()")
    # COLLECTs and DEGRADEs only ever add structural bytes, never remove them.
    if s["sent_collect"] and s["secbytes_struct"] <= 0:
        bad.append(f"secbytes_struct {s['secbytes_struct']:.0f} <= 0 with"
                   f" {s['sent_collect']:.0f} COLLECTs sent")
    # The scalars and the txBytesSec vector are two independent paths over the
    # same model, and they HAVE disagreed: link authentication is charged per
    # PDU including Hello, which never reaches countGpSend, so a vector written
    # only there silently omitted every idle window. Compare them when the
    # vector is on disk and scavetool is around.
    bad += _check_vector_agrees(d, s)
    if not bad:
        sec = s["secbytes_sig"] + s["secbytes_struct"] + s["secbytes_link"]
        print(f"ok  {d}: {s['bytes_total'] / 1e6:.1f} MB -> "
              f"{(s['bytes_total'] + sec) / 1e6:.1f} MB secured "
              f"(x{(s['bytes_total'] + sec) / s['bytes_total']:.2f}; "
              f"sig {s['secbytes_sig'] / sec:.0%} struct {s['secbytes_struct'] / sec:.0%} "
              f"link {s['secbytes_link'] / sec:.0%})")
    return bad


if __name__ == "__main__":
    dirs = sys.argv[1:]
    if not dirs:
        sys.exit(__doc__)
    fails = [m for d in dirs for m in check(d)]
    for m in fails:
        print("FAIL", m, file=sys.stderr)
    sys.exit(1 if fails else 0)
