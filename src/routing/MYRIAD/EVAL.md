# Evaluating MyriadGP (path-vector) vs MyriadLS (link-state)

**Key finding:** GP's message cost tracks the *dimensionality* of the
ring-adjacency graph (rings = nodes, edge = two rings share a router), **not**
node or ring count. 1-D chain → linear, GP wins. 2-D mesh → super-linear, GP
loses. Ring *size* is cheap; path *diversity* is what explodes.

## Ground rules (or the numbers lie)

Both protocols, identical conditions: `sim-time-limit=10s`,
`helloInterval=2s deadInterval=4s rxmtInterval=1s`, `helloJitter=0s` (LS's DBD
exchange wedges under jitter), `nodeM{in,ax}ControlPlaneDelay=1000000ns` (1 ms/hop;
GP livelocks at zero delay), express mode. Metric = sum of per-node `sent_total`
scalars (application-level sends; excludes timers/wrappers/dups).

## Script

`eval_scaling.py` generates each topology + ring labels, writes a self-contained
`.ini` (absolute paths, ring labels inlined — no fragile `include`), runs Cmdenv,
sums `sent_total`, plots. All output in git-ignored `eval/`; artifacts cached
(delete `eval/` to rebuild).

```
python3 eval_scaling.py                # 4 sweeps + plots + topology picture
python3 eval_scaling.py --quick        # short smoke test
python3 eval_scaling.py --only flower  # one of: square|width|ladder|flower|topos
```

Four sweeps + one diagram:

| sweep  | shape (ring-graph)        | grows        | GP result            |
|--------|---------------------------|--------------|----------------------|
| ladder | 2×W, pure **1-D line**    | length       | **best**: flat ~0.45× |
| flower | N petals of N, **star**   | ring **size**| wins, 0.43×→0.31×    |
| width  | 3×W, 2-row mesh           | one dim      | loses, quasi-linear  |
| square | N×N, dense **2-D mesh**   | both dims    | **worst**: 31× @10×10 |

`--only topos` draws one representative of each (pie-node = ring memberships) to
`eval/plots/topologies.png`; scaling plots to `eval/plots/{square,width,ladder,flower}_scaling.png`.

## Results (clean baseline)

| topology       | LS     | GP      | GP/LS |
|----------------|-------:|--------:|------:|
| ladder 2×30    | 9,428  | 4,476   | 0.47× |
| flower N=12    | 13,092 | 4,080   | 0.31× |
| flower (myriad_large, 50n) | 3,315 | 1,030 | 0.31× |
| 3×3 grid       | 458    | 264     | 0.58× |
| width 3×30     | 16,715 | 217,014 | 13×   |
| square 10×10   | 14,142 | 440,400 | 31×   |

**Reading:** GP beats LS on sparse/1-D/star structures (the realistic case) and
only loses on 2-D ring-meshes (grids). 10×10 decomposes exactly:
437,952 updates = 109,407 distinct ring-chains × 4 (each laps its 4-node ring)
— textbook BGP path-hunting, geometric in grid size.

## Manual single comparison

```
cd ../../simulations
../src/RoutingSim -u Cmdenv MYRIAD/myriadls.ini -c PaperTopology '--**.helloJitter=0s' --cmdenv-express-mode=true
../src/RoutingSim -u Cmdenv MYRIAD/myriadgp.ini -c PaperTopology '--**.helloJitter=0s' --cmdenv-express-mode=true
```
Configs in both `.ini`s: `PaperTopology`, `LargeTopology`, `Grid5`, `Grid10`,
`Topo10` (`myriadgp.ini` also `TwoNodeRing`).
