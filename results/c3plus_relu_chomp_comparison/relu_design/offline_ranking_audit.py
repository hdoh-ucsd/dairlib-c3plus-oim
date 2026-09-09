#!/usr/bin/env python3
"""Checkpoint B offline ranking audit: footprint-ReLU vs center-exp obstacle cost.

Synthetic candidate sets around recorded poses (no per-candidate logs exist).
"""
import csv, math, os, glob
import numpy as np
import yaml

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
RUNS = os.path.join(WT, "results/xarm6_c3plus_scene_smoke/runs")
CFG = os.path.join(WT, "tools/scene_smoke/scene_configs")
OUT = os.path.join(WT, "results/c3plus_relu_chomp_comparison/relu_design")

SCENES = ["single_obstacle", "shelf_gap", "ycb_clutter", "slalom", "icra_sign"]

# Center-based fallback discs [x, y, r] per scenario_params.yaml (incl. robot base).
DISCS = {
    "single_obstacle": [[0.35, 0.0, 0.0707], [0.0, 0.0, 0.09]],
    "shelf_gap": [[0.62, -0.053, 0.133], [0.62, 0.0, 0.133], [0.62, 0.053, 0.133],
                  [0.24, -0.04, 0.064], [0.24, 0.04, 0.064], [0.0, 0.0, 0.09]],
    "ycb_clutter": [[0.35, 0.0, 0.0707], [0.22, 0.20, 0.0996], [0.60, -0.16, 0.0520],
                    [0.15, -0.18, 0.0491], [0.0, 0.0, 0.09]],
    "slalom": [[0.05, 0.22, 0.10198], [0.57, 0.22, 0.18111], [0.245, 0.0, 0.09708],
               [0.665, 0.0, 0.08732], [0.05, -0.22, 0.10198], [0.57, -0.22, 0.18111],
               [0.0, 0.0, 0.09]],
    "icra_sign": [[0.5, -0.55, 0.0533], [0.5, -0.25, 0.0719], [0.5, -0.10, 0.0755],
                  [0.5, 0.15, 0.0663], [0.5, 0.30, 0.0515], [0.5, 0.45, 0.0663],
                  [0.5, 0.60, 0.0583], [0.0, 0.0, 0.09]],
}

W_OLD, DECAY = 5000.0, 0.04
W_NEW, MARGIN = 200.0, 0.01
W_POS, W_YAW = 10000.0, 510.0
N_STEPS = 200

PERTS = [(0.02, 0, 0), (-0.02, 0, 0), (0, 0.02, 0), (0, -0.02, 0),
         (0, 0, 0.2), (0, 0, -0.2), (0.02, 0.02, 0.2), (-0.02, -0.02, -0.2)]


def sample_boundary(poly, spacing=0.002):
    pts = []
    n = len(poly)
    for i in range(n):
        a = np.array(poly[i]); b = np.array(poly[(i + 1) % n])
        L = np.linalg.norm(b - a)
        k = max(1, int(math.ceil(L / spacing)))
        for j in range(k):
            pts.append(a + (b - a) * (j / k))
    return np.array(pts)  # (M,2)


def poly_sdf(pts, poly):
    """Signed distance from points (M,2) to polygon (negative inside)."""
    P = np.asarray(poly)
    n = len(P)
    M = pts.shape[0]
    dmin = np.full(M, np.inf)
    inside = np.zeros(M, dtype=bool)
    x, y = pts[:, 0], pts[:, 1]
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        e = b - a
        w = pts - a
        t = np.clip((w @ e) / (e @ e), 0.0, 1.0)
        proj = a + t[:, None] * e
        dmin = np.minimum(dmin, np.linalg.norm(pts - proj, axis=1))
        cond = ((a[1] <= y) & (b[1] > y)) | ((b[1] <= y) & (a[1] > y))
        xin = a[0] + (y - a[1]) / (b[1] - a[1] + 1e-300) * e[0]
        inside ^= cond & (x < xin)
    return np.where(inside, -dmin, dmin)


def disc_sdf(pts, c):
    return np.linalg.norm(pts - np.array(c[:2]), axis=1) - c[2]


class Scene:
    def __init__(self, name):
        with open(os.path.join(CFG, name + ".yaml")) as f:
            cfg = yaml.safe_load(f)
        self.goal = cfg["goal"]
        self.fp_pts = sample_boundary(cfg["footprint"], cfg.get("boundary_sample_spacing", 0.002))
        obs = cfg.get("obstacles", {})
        self.polys = obs.get("polygons", []) or []
        self.discs = obs.get("discs", []) or []
        self.fb_discs = DISCS[name]

    def fp_clearance(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        pts = self.fp_pts @ R.T + np.array([x, y])
        d = np.inf
        for P in self.polys:
            d = min(d, poly_sdf(pts, P).min())
        for D in self.discs:
            d = min(d, disc_sdf(pts, D).min())
        return d

    def center_clearances(self, x, y):
        return [math.hypot(x - d[0], y - d[1]) - d[2] for d in self.fb_discs]

    def new_cost(self, x, y, yaw):
        d = self.fp_clearance(x, y, yaw)
        v = max(0.0, (MARGIN - d) / MARGIN)
        return W_NEW * v * v, d


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def spearman(a, b):
    ra = np.argsort(np.argsort(a)); rb = np.argsort(np.argsort(b))
    ra = ra - ra.mean(); rb = rb - rb.mean()
    den = math.sqrt((ra @ ra) * (rb @ rb))
    return float(ra @ rb / den) if den > 0 else 1.0


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    summary = {}
    sanity_viol = 0
    nan_count = 0
    flagship = None
    for scene in SCENES:
        S = Scene(scene)
        sp_old, sp_base = [], []
        t1_old = t1_base = 0
        t3_old, t3_base = [], []
        nstep = 0
        mismatch = 0
        for pdir in sorted(glob.glob(os.path.join(RUNS, scene, "pair*"))):
            csvs = glob.glob(os.path.join(pdir, "*_metrics.csv"))
            if not csvs:
                continue
            with open(csvs[0]) as f:
                data = list(csv.DictReader(f))
            idx = np.unique(np.linspace(0, len(data) - 1, N_STEPS).astype(int))
            for i in idx:
                r = data[i]
                x, y, yaw = float(r["object_x"]), float(r["object_y"]), float(r["object_yaw"])
                gx, gy, gyaw = float(r["goal_x"]), float(r["goal_y"]), float(r["goal_yaw"])
                t = float(r["sim_time"])
                cands = [(x, y, yaw)] + [(x + dx, y + dy, yaw + dw) for dx, dy, dw in PERTS]
                task = np.array([W_POS * ((cx - gx) ** 2 + (cy - gy) ** 2)
                                 + W_YAW * wrap(cw - gyaw) ** 2 for cx, cy, cw in cands])
                old = np.array([sum(W_OLD * math.exp(-dc / DECAY)
                                    for dc in S.center_clearances(cx, cy))
                                for cx, cy, cw in cands])
                new = np.zeros(9); dfp = np.zeros(9)
                for k, (cx, cy, cw) in enumerate(cands):
                    new[k], dfp[k] = S.new_cost(cx, cy, cw)
                    if dfp[k] >= MARGIN and new[k] != 0.0:
                        sanity_viol += 1
                for arr in (task, old, new, dfp):
                    nan_count += int(np.isnan(arr).sum())
                A = task + old      # comparator 1: old exp term
                B = task + new      # variant
                C = task            # comparator 2: true frozen baseline (no obstacle term)
                sp_old.append(spearman(A, B)); sp_base.append(spearman(C, B))
                aA, aB, aC = int(np.argmin(A)), int(np.argmin(B)), int(np.argmin(C))
                ch_old = aA != aB; ch_base = aC != aB
                t1_old += ch_old; t1_base += ch_base
                topB = set(np.argsort(B)[:3])
                t3_old.append(len(topB & set(np.argsort(A)[:3])) / 3.0)
                t3_base.append(len(topB & set(np.argsort(C)[:3])) / 3.0)
                nstep += 1
                dcen = min(S.center_clearances(x, y))
                mm = (dcen > 0.025) and (dfp[0] < 0.010)
                mismatch += mm
                pair = os.path.basename(pdir)
                rows.append([scene, pair, t, f"{old[0]:.4f}", f"{new[0]:.4f}",
                             f"{dcen:.4f}", f"{dfp[0]:.4f}", int(ch_old), int(ch_base), int(mm)])
                if scene == "single_obstacle" and pair == "pair01" and abs(t - 316.0) < 1.5:
                    flagship = dict(t=t, d_center=dcen, d_fp=dfp[0], old=old[0], new=new[0],
                                    top1_changed_vs_base=ch_base, top1_changed_vs_old=ch_old)
        summary[scene] = dict(
            n=nstep,
            spearman_old=float(np.mean(sp_old)), spearman_base=float(np.mean(sp_base)),
            top1_old=t1_old / nstep, top1_base=t1_base / nstep,
            top3_old=float(np.mean(t3_old)), top3_base=float(np.mean(t3_base)),
            mismatch=mismatch / nstep)
        print(scene, summary[scene])

    with open(os.path.join(OUT, "offline_candidate_ranking.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scene", "pair", "sim_time", "old_obs_cost_recorded", "new_obs_cost_recorded",
                    "d_center_min", "d_fp", "top1_changed_vs_old", "top1_changed_vs_baseline",
                    "center_safe_fp_unsafe"])
        w.writerows(rows)

    print("sanity_violations:", sanity_viol, "nan_count:", nan_count)
    print("flagship:", flagship)
    import json
    with open(os.path.join(OUT, "_summary.json"), "w") as f:
        json.dump(dict(summary=summary, sanity_violations=sanity_viol,
                       nan_count=nan_count, flagship=flagship), f, indent=1)


if __name__ == "__main__":
    main()
