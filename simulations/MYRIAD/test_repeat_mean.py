#!/usr/bin/env python3
"""Self-check for eval_harness' repeat aggregation (_agg / repeat_mean / Stat)."""
import eval_harness as H


def main():
    # numbers -> Stat(mean, population stdev)
    v = H._agg([1, 2, 3])
    assert v == 2.0 and abs(v.sd - 0.816496580927726) < 1e-12, v

    # nested dict + tuple, elementwise
    r = H._agg([{"a": (1, 10.0), "b": {2: 4}},
                {"a": (3, 20.0), "b": {2: 6}}])
    assert r["a"][0] == 2.0 and r["a"][1] == 15.0 and r["b"][2] == 5.0, r
    assert r["a"][0].sd == 1.0, r["a"][0].sd

    # dicts keep only keys present in ALL reps (ragged snapshot grids)
    assert H._agg([{1: 1, 2: 2}, {1: 3}]) == {1: 2.0}

    # non-numerics pass through as rep 0's value
    assert H._agg(["ls", "gp"]) == "ls"
    assert H._agg([None, None]) is None

    # repeats==1 passes the raw result through untouched (ints stay ints)
    H.REPEATS = 1
    assert H.repeat_mean(lambda rep: {"n": 7}) == {"n": 7}

    # repeats>1 runs each rep once, in parallel, with distinct rep indices
    H.REPEATS = 4
    seen = []
    out = H.repeat_mean(lambda rep: (seen.append(rep), {"n": rep})[1])
    assert sorted(seen) == [0, 1, 2, 3], seen
    assert out["n"] == 1.5, out

    # per-repeat paths never collide
    assert H.rep_path("res", 0) == "res" and H.rep_path("res", 2) == "res.r2"

    # ± suffix only for aggregated values
    assert H.pm(H.Stat(1.0, 0.25)) == "±0.2" and H.pm(1.0) == ""

    print("test_repeat_mean: OK")


if __name__ == "__main__":
    main()
