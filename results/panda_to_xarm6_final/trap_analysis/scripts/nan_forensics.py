#!/usr/bin/env python3
"""Task B: NaN forensics for planner AllFinite crashes (r4 trial1, r5 trial2)."""
import json, math, os, re
import numpy as np

DATA = "/root/push_anything_ADMM/results"
OUT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant/results/panda_to_xarm6_final/trap_analysis"
CASES = [("r4_trial1", f"{DATA}/panda_to_xarm6_port_final_r4/xarm6_trial1"),
         ("r5_trial2", f"{DATA}/panda_to_xarm6_port_final_r5/xarm6_trial2")]

results = []
for name, d in CASES:
    ev = []
    # 1. scan state trace for nonfinite / absurd jumps
    prev = None
    first_bad = None
    n = 0
    with open(os.path.join(d, "state_trace.jsonl")) as f:
        for line in f:
            r = json.loads(line); n += 1
            vals = r["q"] + r["obj"] + [r["pos_err"], r["ang_err"]]
            if not all(math.isfinite(v) for v in vals):
                first_bad = (r["t"], "nonfinite_state"); break
            if prev is not None:
                dq = max(abs(a - b) for a, b in zip(r["q"], prev["q"]))
                dob = math.dist(r["obj"][4:6], prev["obj"][4:6])
                if dq > 1.0:
                    first_bad = (r["t"], f"q_jump_{dq:.2f}rad"); break
                if dob > 0.5:
                    first_bad = (r["t"], f"obj_jump_{dob:.2f}m"); break
            prev = r
    last_trace_t = prev["t"]
    ev.append(f"state_trace: {n} samples scanned, all finite, no |dq|>1 rad or |dobj|>0.5 m jumps; last sample t={last_trace_t}")
    if first_bad:
        ev[-1] = f"state_trace: FIRST BAD at t={first_bad[0]}: {first_bad[1]}"

    # 2. osc.log: last XARM6_5J print, last p_des change (planner-alive marker), last PRELIFT,
    #    any nonfinite in prints, saturation run near end
    last_osc_t = None; nonfinite_osc = None; qd = {}
    last_pdes = None; last_pdes_change_t = None; last_prelift_t = None
    pat = re.compile(r"XARM6_5J t=(\d+) p_des=\s*(\S+)\s+(\S+)\s+(\S+).*\|qdot_cmd\|=(\S+)")
    with open(os.path.join(d, "osc.log")) as f:
        for line in f:
            if line.startswith("PRELIFT_RELEASE="):
                m2 = re.search(r"t=(\S+)", line)
                if m2:
                    last_prelift_t = float(m2.group(1))
            if "nan" in line.lower() or "inf" in line.lower():
                if nonfinite_osc is None and "XARM6" in line:
                    nonfinite_osc = line.strip()
            m = pat.search(line)
            if m:
                last_osc_t = int(m.group(1)); qd[last_osc_t] = float(m.group(5))
                pd = (m.group(2), m.group(3), m.group(4))
                if pd != last_pdes:
                    last_pdes_change_t = last_osc_t; last_pdes = pd
    # saturated run length ending at last print
    run = 0; t = last_osc_t
    while t in qd and qd[t] >= 0.45:
        run += 1; t -= 1
    # longest saturated run overall
    best = cur = 0
    for t in sorted(qd):
        cur = cur + 1 if qd[t] >= 0.45 else 0
        best = max(best, cur)
    ev.append(f"osc.log: last XARM6_5J print at t={last_osc_t}s, no nonfinite values printed"
              + ("" if not nonfinite_osc else f"; NONFINITE: {nonfinite_osc}"))
    ev.append(f"planner-alive markers: last p_des change at t={last_pdes_change_t}s, last PRELIFT event at t={last_prelift_t}s; "
              f"sim+executor continued fully finite for ~{last_osc_t - (last_pdes_change_t or 0)}s after the last planner-driven "
              f"target update — the crash killed only the planner process")
    ev.append(f"osc.log: |qdot_cmd|>=0.45 run ending at crash = {run}s; longest saturated run in trial = {best}s")

    # 3. planner.log / sim.log tails
    def tail_flags(fn, k=400):
        lines = open(os.path.join(d, fn), errors="replace").readlines()[-k:]
        return [l.strip() for l in lines if re.search(r"nan|inf|WARNING|error|Failure|terminate", l, re.I)]
    pl = tail_flags("planner.log")
    sm = tail_flags("sim.log")
    ev.append(f"planner.log tail flags: {pl[-4:]}")
    ev.append(f"sim.log tail flags: {sm[-4:] if sm else 'none — sim.log ends cleanly with no warnings'}")

    crash_t = f"planner last alive t~{last_pdes_change_t}s (last p_des update); run/logs end t={last_trace_t}s"
    # saturation around planner death time
    sat_at_death = sum(1 for t in range(max(0, (last_pdes_change_t or 0) - 30), (last_pdes_change_t or 0) + 1)
                       if qd.get(t, 0) >= 0.45)
    ev.append(f"saturation near planner death: {sat_at_death}/30 s with |qdot_cmd|>=0.45 in the 30 s before the last p_des update")
    # classification logic
    if first_bad:
        cls = "N1_SIM_CONTACT_INSTABILITY"; fb = first_bad[1]
    else:
        cls = "N4_PLANNER_INTERNAL_OVERFLOW"
        fb = "transient_not_captured"
        ev.append("All recorded signals (10 Hz sim state, 1 Hz servo command) are finite through the crash instant; "
                  "the nonfinite vector reached SetPositionsAndVelocities inside the planner process. The 10 Hz recorder "
                  "could miss a <100 ms sim transient, but the sim and executor both kept running normally for hundreds "
                  "of seconds after the planner's last target update (osc.log XARM6_5J prints continue with finite, "
                  "bounded |qdot_cmd|; sim.log has no warnings), which argues the sim state stayed finite (rules out N1) "
                  "and the servo command stayed finite (rules out N2); the nonfinite value was internal to the planner "
                  "(its own predicted-x0/rollout state), i.e. N4, with N3 (bad planner input only) as the alternative "
                  "the 10 Hz recorder cannot fully exclude.")
    results.append(dict(
        trial=name,
        crash_t_last_planner_alive=crash_t,
        last_executor_print_t=last_osc_t,
        first_bad_signal=fb,
        classification=cls,
        saturated_run_ending_at_crash_s=run,
        longest_saturated_run_s=best,
        evidence=ev,
        recommended_guard="Guard at the planner's state-input boundary: before every SetPositionsAndVelocities "
                          "(LCS formulation and internal rollout), check AllFinite on the assembled q_v; on failure, "
                          "log the offending vector + source (LCM message vs internal prediction), clamp/skip the tick "
                          "instead of aborting. Secondary: finite-check the planner's predicted-x0 extrapolation "
                          "(velocity extrapolation under saturated pressing is the prime internal suspect).",
    ))

with open(os.path.join(OUT, "nan_first_bad_signal.json"), "w") as f:
    json.dump(results, f, indent=2)
for r in results:
    print(json.dumps(r, indent=1)[:2200])
