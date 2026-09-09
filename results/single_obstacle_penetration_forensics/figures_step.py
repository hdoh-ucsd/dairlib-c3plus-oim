#!/usr/bin/env python3
import numpy as np, math, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly, Rectangle, Circle

OUT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics/results/single_obstacle_penetration_forensics"
c = np.load(f"{OUT}/geometry/_cache.npz")
cm = np.load(f"{OUT}/geometry/_cache_metrics.npz")
t, d, phi_c, phi_f, xy, yaw = c["t"], c["d"], c["phi_c"], c["phi_f"], c["xy"], c["yaw"]
quat = c["quat"]; z = c["z"]
ev = dict(zip(["T0", "T1", "T2", "T3", "T4"], c["events"]))

FOOT = [(-0.0445, 0.0198), (-0.0445, 0.0), (-0.0099, 0.0), (-0.0099, -0.0794),
        (0.0099, -0.0794), (0.0099, 0.0), (0.0445, 0.0), (0.0445, 0.0198)]

def quat_R(q):
    w, x, y, zz = q
    return np.array([
        [1-2*(y*y+zz*zz), 2*(x*y-w*zz), 2*(x*zz+w*y)],
        [2*(x*y+w*zz), 1-2*(x*x+zz*zz), 2*(y*zz-w*x)],
        [2*(x*zz-w*y), 2*(y*zz+w*x), 1-2*(x*x+y*y)]])

# 3D collision boxes of T (from SDF): crossbar center (0,0.0099,0) size .089x.0198x.0596
# stem: link pose (0,-0.0397,0), box .0198x.0794x.0596
BOXES = [((0, 0.0099, 0), (0.089, 0.0198, 0.0596)),
         ((0, -0.0397, 0), (0.0198, 0.0794, 0.0596))]

def box_outline_xy(R, p, center, size):
    h = np.array(size) / 2
    corners = np.array([[sx*h[0], sy*h[1], sz*h[2]] for sx in (-1, 1)
                        for sy in (-1, 1) for sz in (-1, 1)]) + np.array(center)
    W = (R @ corners.T).T + p
    pts = W[:, :2]
    from scipy.spatial import ConvexHull
    hull = ConvexHull(pts)
    return pts[hull.vertices]

for name, i in ev.items():
    if i < 0:
        continue
    i = int(i)
    fig, ax = plt.subplots(figsize=(7, 7))
    # obstacle visual + collision are identical 0.1x0.1 box
    ax.add_patch(Rectangle((0.30, -0.05), 0.1, 0.1, fill=True, alpha=0.25,
                           color="tab:orange", label="obstacle visual (0.1x0.1)"))
    ax.add_patch(Rectangle((0.30, -0.05), 0.1, 0.1, fill=False, ls="--",
                           color="darkred", lw=2, label="obstacle Drake collision"))
    ax.add_patch(Circle((0, 0), 0.09, fill=False, color="gray", ls=":",
                        label="base disc (planner)"))
    R = quat_R(quat[i] / np.linalg.norm(quat[i]))
    p = np.array([xy[i, 0], xy[i, 1], z[i]])
    # T visual footprint (planner footprint polygon at yaw)
    cs, sn = math.cos(yaw[i]), math.sin(yaw[i])
    Fp = np.array([[xy[i, 0]+cs*a-sn*b, xy[i, 1]+sn*a+cs*b] for a, b in FOOT])
    ax.add_patch(MPoly(Fp, closed=True, fill=True, alpha=0.3, color="tab:blue",
                       label="T visual / planner footprint"))
    for k, (ctr, sz2) in enumerate(BOXES):
        o = box_outline_xy(R, p, ctr, sz2)
        ax.add_patch(MPoly(o, closed=True, fill=False, color="navy", lw=2,
                           label="T Drake collision (proj)" if k == 0 else None))
    ax.set_xlim(0.15, 0.65); ax.set_ylim(-0.25, 0.25); ax.set_aspect("equal")
    ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"{name}  t={t[i]:.1f}s  d_phys={d[i]*1000:.2f}mm  "
                 f"phi_center={phi_c[i]*1000:.1f}mm  phi_footprint={phi_f[i]*1000:.2f}mm")
    fig.savefig(f"{OUT}/figures/{name}_geometry_overlay.png", dpi=130,
                bbox_inches="tight")
    plt.close(fig)

fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t, d * 1000, label="d_phys (Drake, 3D)", lw=1.5)
ax.plot(t, phi_c * 1000, label="phi_planner_center (ranking)", lw=1)
ax.plot(t, phi_f * 1000, label="phi_planner_footprint (route/LCS)", lw=1, ls="--")
ax.axhline(0, color="k", lw=0.8)
ax.set_yscale("symlog", linthresh=1)
ax.set_xlabel("t [s]"); ax.set_ylabel("signed distance [mm, symlog]")
ax.legend(); ax.grid(alpha=0.3)
ax.set_title("Object-obstacle signed distance vs time (never < 0)")
fig.savefig(f"{OUT}/figures/signed_distance_vs_time.png", dpi=130, bbox_inches="tight")
plt.close(fig)

fig, ax = plt.subplots(figsize=(9, 5))
diff = cm["phi_f_m"] - cm["d_m"]
ax.plot(cm["mst"], diff * 1000, lw=0.7)
ax.axhline(0, color="k", lw=0.8)
ax.set_xlabel("sim_time [s]"); ax.set_ylabel("phi_footprint - d_phys [mm]")
ax.set_title("Planner 2D footprint SDF minus Drake 3D signed distance")
ax.grid(alpha=0.3)
fig.savefig(f"{OUT}/figures/physical_vs_planner_gap.png", dpi=130, bbox_inches="tight")
plt.close(fig)
print("figures done")
