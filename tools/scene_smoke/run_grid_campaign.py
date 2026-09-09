#!/usr/bin/env python3
"""Grid campaign orchestrator for the baseline-vs-ReLU comparison.

Runs (scene, start M, goal N) trials through a fixed number of lanes using
run_scene_smoke.sh (isolated ports/dirs/pgroups). Variant 'relu' adds the
env-gated ranking term; 'baseline' runs the default (compile-identical
frozen) path.

Usage: run_grid_campaign.py --variant baseline|relu --pairs offdiag|all|smoke
       [--cap 600] [--lanes 2] [--port-base 8000]
"""
import argparse, os, re, shutil, subprocess, time

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
S = os.path.join(WT, "tools/scene_smoke")
OUTROOT = os.path.join(WT, "results/c3plus_relu_chomp_comparison")
FAM = {
    "open_task": "matched_open_table_xarm6_",
    "single_obstacle": "matched_single_obstacle_xarm6_",
    "shelf_gap": "matched_shelf_gap_xarm6_",
    "ycb_clutter": "matched_ycb_clutter_xarm6_",
    "slalom": "matched_slalom_xarm6_",
    "icra_sign": "anything_icra_c_matched_xarm6_",
}
ENVF = {
    "open_task": "", "single_obstacle": f"{S}/env_single_obstacle.sh",
    "shelf_gap": f"{S}/env_shelf_gap.sh", "ycb_clutter": f"{S}/env_ycb_clutter.sh",
    "slalom": f"{S}/env_slalom.sh", "icra_sign": f"{S}/env_icra_sign.sh",
}
SCENES = list(FAM)


def demo_name(scene, m, n):
    return FAM[scene] + (f"t{m}" if m == n else f"s{m}g{n}")


def goal_of(scene, n):
    gp = os.path.join(WT, "examples/sampling_c3", FAM[scene] + f"t{n}",
                      "parameters/goal_params.yaml")
    txt = open(gp).read()
    pos = re.search(r"^fixed_target_position: \[([^\]]+)\]", txt, re.M).group(1)
    quat = re.search(r"^fixed_target_orientation: \[([^\]]+)\]", txt, re.M).group(1)
    x, y, _ = [float(v) for v in pos.split(",")]
    import math
    w, _, _, z = [float(v) for v in quat.split(",")]
    yaw = 2.0 * math.atan2(z, w)
    return x, y, yaw


def jobs_for(pairs):
    out = []
    for scene in SCENES:
        for m in range(1, 6):
            for n in range(1, 6):
                if pairs == "offdiag" and m == n:
                    continue
                if pairs == "smoke" and (scene, m, n) not in [
                        ("single_obstacle", 1, 1), ("single_obstacle", 3, 3),
                        ("shelf_gap", 1, 1), ("slalom", 1, 1)]:
                    continue
                out.append((scene, m, n))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", required=True, choices=["baseline", "relu"])
    ap.add_argument("--pairs", required=True, choices=["offdiag", "all", "smoke"])
    ap.add_argument("--cap", type=int, default=600)
    ap.add_argument("--lanes", type=int, default=2)
    ap.add_argument("--port-base", type=int, default=8000)
    a = ap.parse_args()

    env = dict(os.environ)
    if a.variant == "relu":
        env["SAMPLING_C3_RANK_OBS_MODE"] = "relu_footprint"
        env["SAMPLING_C3_OBS_RELU_EPS"] = "0.01"
        env["SAMPLING_C3_OBS_RELU_W"] = "200"

    jobs = jobs_for(a.pairs)
    print(f"{len(jobs)} jobs, variant={a.variant}, cap={a.cap}, lanes={a.lanes}",
          flush=True)
    running = []  # (proc, label)
    port = a.port_base
    for scene, m, n in jobs:
        while len(running) >= a.lanes:
            for p, lab in list(running):
                if p.poll() is not None:
                    print(f"[DONE] {lab} rc={p.returncode}", flush=True)
                    running.remove((p, lab))
            time.sleep(5)
        out = os.path.join(OUTROOT, a.variant, scene, f"s{m:02d}g{n:02d}")
        # Resume support: a completed run has "RUN DONE" in launcher.log —
        # skip it. A partial dir (crash mid-run) is wiped and rerun.
        lg = os.path.join(out, "launcher.log")
        if not os.path.islink(out) and os.path.isfile(lg) and \
                "RUN DONE" in open(lg, errors="replace").read():
            print(f"[SKIP done] {a.variant}/{scene}/s{m}g{n}", flush=True)
            continue
        if os.path.isdir(out) and not os.path.islink(out) and os.listdir(out):
            print(f"[WIPE partial] {a.variant}/{scene}/s{m}g{n}", flush=True)
            shutil.rmtree(out)
        os.makedirs(out, exist_ok=True)
        gx, gy, gyaw = goal_of(scene, n)
        port += 1
        lab = f"{a.variant}/{scene}/s{m}g{n}"
        print(f"[LAUNCH] {lab} port {port} goal {gx} {gy} {gyaw:.4f}", flush=True)
        lf = open(os.path.join(out, "launcher.log"), "w")
        p = subprocess.Popen(
            ["bash", f"{S}/run_scene_smoke.sh", demo_name(scene, m, n),
             "G_shape_video", str(gx), str(gy), f"{gyaw:.6f}", str(a.cap),
             str(port), out, ENVF[scene]],
            stdout=lf, stderr=subprocess.STDOUT, env=env)
        running.append((p, lab))
    while running:
        for p, lab in list(running):
            if p.poll() is not None:
                print(f"[DONE] {lab} rc={p.returncode}", flush=True)
                running.remove((p, lab))
        time.sleep(5)
    print("CAMPAIGN COMPLETE", flush=True)


if __name__ == "__main__":
    main()
