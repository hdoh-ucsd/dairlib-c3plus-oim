# OIM per-control-step evaluation figure — source-of-truth definitions

Authority: upstream OIM @ d6d80a6, `/root/push_anything_ADMM/external/oim_upstream_main`.
The figure is drawn by `oim/utils/plotting.py` (`plot_run_3d` -> `_diagnostics_panel` +
`_cost_panel(title="Robot block costs")`); the series come from
`oim/utils/costs.py:cost_series` and `oim/utils/metrics.py:goal_errors`.

## LEFT panel — "Task diagnostics" (`oim/utils/plotting.py:108-159`)

- **position error (m)** — `oim/utils/metrics.py:28-46 (goal_errors)`:
  `pos_err[k] = || object_pose[1:][k, :2] - goal[:2] ||_2`
  i.e. Euclidean distance of the object's planar position **after** control step k
  from the goal xy.
- **orientation error (rad)** — same function:
  `theta_err[k] = | wrap(object_pose[1:][k, 2] - goal[2]) |`
  with `wrap(a) = (a + pi) mod 2pi - pi`, i.e. wrapped to **(-pi, pi]**
  (`metrics.py:23-25`; identical to `oim/objects/planar_pushing.py:37-39 wrap_angle`).
- **x-axis: control step index** (`ax.set_xlabel("control step")`, plotting.py:134/145),
  NOT wall time. Index k corresponds to the state produced by control step k
  (state series carry the initial condition, hence the `[1:]` shift —
  `costs.py:388-403`, `metrics.py:33-46`).
- ADMM runs additionally overlay `primal_residual`, `dual_residual`, `rho` on the left
  axis; the two goal errors then move to a twinned right axis (dashed lines).
  Flat/C3 runs plot only the two errors.

## RIGHT panel — "Robot block costs" (`plotting.py:304-389, 424-451`)

Series = `costs.cost_series(task, log)` in `TERM_ORDER` (`costs.py:36-51`):
`goal_pos, goal_theta, obstacle, support, rate*, approach, align, tilt, tip_z,
contact_z, pusher_obstacle, robot_contact, effort, admm_penalty`
(*`rate` appears only in the separate "Object block costs" panel.*)
Every series is a **passive recomputation from the finished log** ("nothing here is
on the hot path", costs.py:16-19); each mirrors a LIVE optimizer term (paper
eq. 20-22 for approach/align/effort), except `robot_contact`, which is plotted at
execution fidelity while the optimizer weights a planning-fidelity force.

Rendering (plotting.py:344-389): per-step value (NOT accumulated), each term drawn
faint raw + rolling mean (window = clamp(n/40, 1, 25)); terms whose run-total is 0
are dropped; y-axis is symlog with linthresh = max(min median of positive values,
1e-3); legend shows `name (Σ total)`.

### `total`
No explicit total series is stored. The panel plots
`total[k] = sum over ACTIVE terms of term[k]` (black dashed, plotting.py:366-374),
and `costs.cost_totals` (costs.py:750-754) defines the scalar
`total = sum_k sum_terms term[k]`. **evaluation_total for C3+** should be the per-step
sum over the *mappable, non-NaN* blocks, with the member set stated in the legend.

## Weights (xarm6, `oim/configs/robots/xarm6.yaml`; DEFAULT_COSTS in `oim/tasks/pusht.py:83-300` in parentheses)

q_pos 200 (40) | q_theta 16 (10) | q_ramp_per_step 0.005 (0) | q_ramp_max 30 (5) |
w_obstacle 50 (10), obstacle_decay 0.10 (0.02) | w_support 500 (200), support_margin 0.2 (0.10) |
w_robot_effort 0.05 | w_approach 60 (40), r0 0.075 (0.02) | w_align 50 (15), gamma0 60deg (15) |
w_tilt 80 (30) | w_z_tip 1.0 (0.0008), w_z_tip_exp 1.0, tip_floor_z 0.012 real / 0 sim,
tip_floor_scale 0.004 | w_contact_z_exp 200 (0), contact_z_slab 0.015 (0.01),
contact_z_margin 0.008 (0) | w_robot_contact 0.1 (1.0) | shaping_fade_dist 0.25 (0) |
EXP_ARG_MAX = 10.0 (pusht.py:66).

## Pseudocode — per-control-step computation for the C3+ evaluator

Inputs per control step k (C3+ log): object quat+xyz -> (x, y, yaw); goal (gx, gy, gyaw);
EE pose -> (ex, ey, ez) and EE quaternion; 5 joint positions q[k]; control_dt;
scene constants: footprint polygon (object frame), tip_target_z (block mid-height),
block_half_height, obstacle shapes, table box (if any).

```
wrap(a)   = mod(a + pi, 2pi) - pi                     # (-pi, pi]
pos_err   = hypot(x - gx, y - gy)
theta_err = abs(wrap(yaw - gyaw))

ramp  = min(1 + 0.005 * k, 30.0)                      # q_ramp, both goal terms
fade  = clamp(pos_err / 0.25, 0, 1)                   # shaping_fade_dist

goal_pos   = ramp * 200.0 * ((x-gx)^2 + (y-gy)^2)
goal_theta = ramp * 16.0  * wrap(yaw - gyaw)^2

# obstacle: sample the footprint boundary (OIM: object.world_boundary sample set),
# transform to world by (x, y, yaw); d_i = signed distance to nearest obstacle
obstacle = 50.0 * sum_i exp(-d_i / 0.10)              # NaN if geometry unavailable
support  = 500.0 * sum_i max(sdf_table(p_i) + 0.2, 0)^2   # NaN if no table region

d2       = (ex-x)^2 + (ey-y)^2
approach = fade * 60.0 * max(d2 - 0.075^2, 0)

cosang = dot((x-ex, y-ey), (gx-x, gy-y)) / (|..|*|..| + 1e-6)
align  = fade * 50.0 * max(cos(60deg) - cosang, 0)
# top-contact gate (align_top_suppress = 1):
local  = R(-yaw) @ (ex - x, ey - y)
top_z  = tip_target_z + block_half_height
if sdf_footprint(local) <= 0 and (top_z - 0.005) <= ez <= (top_z + 0.05):
    align = 0.0

psi  = angle(EE tool "down" axis, world -z)            # from EE quaternion
tilt = fade * 80.0 * (1 - cos(psi))

# tip_z, real-config branch (tip_floor_z = 0.012, tip_floor_scale = 0.004):
quad  = 1.0 * (100 * (max(ez, 0.012) - tip_target_z))^2
gap   = max(0.012 - ez, 0) / 0.004
tip_z = quad + 1.0 * (exp(min(gap^2, 10.0)) - 1.0)
# (sim branch, tip_floor_z = 0: above target fade*w*(100*dz)^2, below
#  exp(min((100*(target-ez))^2, 10)) — pick the branch matching the OIM config.)

dz = ez - top_z
if sdf_footprint(local) <= 0.008 and -0.015 <= dz <= 0.015:
    g = 1 - min(abs(dz)/0.015, 1)
    contact_z = 200.0 * exp((2*g)^2)
else:
    contact_z = 0.0

qdot   = (q[k] - q[k-1]) / control_dt                  # realized-velocity PROXY
effort = fade * 0.05 * sum(qdot^2)                     # NaN at k = 0

pusher_obstacle = NaN   # inert in OIM (weight 0)
robot_contact   = NaN   # no contact-force log in C3+
admm_penalty    = NaN   # ADMM-only
rate            = NaN   # object-block term

evaluation_total = sum of non-NaN blocks above (state the member set)
```

Never substitute 0 for a NaN block: OIM itself omits terms it cannot compute
("absent rather than zero", costs.py:449-451, 103-104). Do not touch the C3+
live objective — this is evaluation instrumentation only.
