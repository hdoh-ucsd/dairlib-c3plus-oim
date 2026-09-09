# CHOMP-style obstacle metric — definition

Evaluation-only functional computed OFFLINE from `state_trace.jsonl` object
poses and the scene geometry in `tools/scene_smoke/scene_configs/<scene>.yaml`.

## Cost function
With eps = 0.01 m and d the min signed distance (m) from a body point to all
scene obstacles (polygon and disc SDFs; negative inside):

    c(d) = -d + eps/2            if d < 0
    c(d) = (d - eps)^2 / (2 eps) if 0 <= d <= eps
    c(d) = 0                     if d > eps

## Body points
A FIXED set of points in the object frame: the footprint polygon boundary of
the scene config, sampled every 5 mm (each edge split into
ceil(edge_len/0.005) equal parts; every vertex included exactly once). The
same set is used for every run of a scene, transformed rigidly by the planar
object pose (x, y, yaw from the state-trace quaternion) at each state k.

## Functional
    M_CHOMP = sum_k sum_u 0.5 * (c(d_{k,u}) + c(d_{k+1,u})) * ||x_{k+1,u} - x_{k,u}||
    M_CHOMP_norm = M_CHOMP / L_center     (0 if L_center = 0)

where x_{k,u} is the world position of body point u at state k and L_center
is the total object CENTER path length. Scenes without obstacles (open_task)
score exactly 0.

## Independence from the controller ranking cost
This metric is NOT the controllers' obstacle ranking term. The baseline
variant ranks with 5000*exp(-(d_center - r)/0.04) on the object CENTER; the
ReLU variant ranks with 200*max(0, (0.01 - d_fp)/0.01)^2 on the footprint.
M_CHOMP differs from both in functional form (piecewise CHOMP potential),
integration (path-length-weighted trapezoid over a dense boundary point set),
and role: it is never fed back to any controller — it is a neutral
evaluation yardstick applied identically to both variants.
