#!/usr/bin/env python3
"""Exact-contact microtests for the ICRA glyphs I/C/R/A (validation brief 9-10).

Deterministic pydrake rig (proven scripted-pusher pattern from
results/c3plus_route_prediction_fidelity/route_value_audit/micro_push_test.py):
  - MultibodyPlant dt 1e-3, ground halfspace mu=0.3
  - kinematic sphere pusher r=0.0195, mu=1.5 (Drake combines 2*mu1*mu2/(mu1+mu2):
    EE-obj 2*1.5*0.3/1.8 = 0.5; obj-ground 0.3 -- matched_mu semantics)
  - glyph SDFs examples/sampling_c3/urdf/push_{i,c,r,a}_glyph.sdf (mu 0.3),
    spawned resting bottom-on-ground (half-thickness 0.0125)
  - pusher starts exactly touching the chosen boundary point at mid-thickness
    height (z=0.0125), driven 0.05 m/s for 2.0 s along the inward normal.

Contacts per glyph, from footprint hulls (icra_glyph_obstacles.csv local
coords; C from push_c_glyph.sdf box union):
  A_translation: through-centroid push on the longest edge (inward normal)
  B_rotation:    max-moment-arm boundary point, along-edge offset >= 60% of
                 the max half-extent from the centroid
  C_mixed:       ~30% along-edge offset on the longest edge
3 reps each (deterministic; spread reported if not identical).
"""
import os
import csv
import numpy as np
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.geometry import Sphere, HalfSpace
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.multibody.math import SpatialVelocity

ROOT = '/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant'
URDF = os.path.join(ROOT, 'examples/sampling_c3/urdf')
OUT = os.path.join(ROOT, 'results/icra_glyph_open_table')

PUSHER_R = 0.0195
PUSHER_MU = 1.5
GROUND_MU = 0.3
SPEED = 0.05
DUR = 2.0
DT = 1e-3
ZMID = 0.0125  # mid-thickness (all glyphs half-thickness 0.0125)

# Footprint hulls, local coords (icra_glyph_obstacles.csv rows for I, R, A)
HULLS = {
    'I': [(-0.032, -0.0149), (0.0503, -0.0149), (0.0511, -0.0145), (0.0515, -0.0137),
          (0.0515, 0.0138), (0.0503, 0.0149), (-0.0504, 0.0149), (-0.0515, 0.0137),
          (-0.0515, -0.0138), (-0.0504, -0.0149)],
    'R': [(0.0515, -0.0496), (0.0515, 0.0496), (0.0511, 0.0506), (-0.0294, 0.0417),
          (-0.0445, 0.034), (-0.0502, 0.0208), (-0.0515, 0.0035), (-0.0515, -0.0496),
          (-0.0507, -0.0506), (0.0508, -0.0506)],
    'A': [(-0.0515, 0.0162), (-0.0515, -0.0162), (-0.0501, -0.018), (0.0499, -0.0556),
          (0.0511, -0.0556), (0.0515, -0.0543), (0.0515, 0.0543), (0.0511, 0.0556),
          (0.0499, 0.0556), (-0.0501, 0.018)],
}


def poly_ccw_centroid(V):
    V = np.array(V, float)
    x, y = V[:, 0], V[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    cr = x * yn - xn * y
    A = 0.5 * cr.sum()
    if A < 0:
        return poly_ccw_centroid(V[::-1])
    cx = ((x + xn) * cr).sum() / (6 * A)
    cy = ((y + yn) * cr).sum() / (6 * A)
    return V, np.array([cx, cy]), A


def pick_contacts(V):
    """Return dict name -> (contact_pt_local, inward_normal_2d, note), centroid."""
    V, c, _ = poly_ccw_centroid(V)
    n = len(V)
    maxhe = max(np.abs(V - c).max(), 1e-9)
    edges = []
    for i in range(n):
        a, b = V[i], V[(i + 1) % n]
        e = b - a
        L = np.linalg.norm(e)
        t = e / L
        nin = np.array([-t[1], t[0]])  # CCW -> left normal points inward
        edges.append((a, b, t, nin, L))
    # A: longest edge, projection of centroid
    a, b, t, nin, L = max(edges, key=lambda e: e[4])
    s = np.clip(np.dot(c - a, t), 0, L)
    pA = a + s * t
    # C: same edge, ~30% of maxhe offset along edge from centroid projection
    s30 = np.clip(s + 0.3 * maxhe, 0.0, L)
    if s30 - s < 0.25 * maxhe:  # room on the other side instead
        s30 = np.clip(s - 0.3 * maxhe, 0.0, L)
    pC = a + s30 * t
    # B: sample all edges, maximize |moment arm| = |cross(p-c, nin)| subject to
    # along-edge offset from centroid >= 60% maxhe
    best = None
    for (a2, b2, t2, nin2, L2) in edges:
        for f in np.linspace(0.02, 0.98, 49):
            p = a2 + f * L2 * t2
            r = p - c
            mom = r[0] * nin2[1] - r[1] * nin2[0]
            off = abs(np.dot(r, t2))
            if off >= 0.6 * maxhe and (best is None or abs(mom) > abs(best[2])):
                best = (p, nin2, mom)
    pB, nB, momB = best
    return {'A_translation': (pA, nin, f'longest edge L={L:.4f}'),
            'B_rotation': (pB, nB, f'moment_arm={momB:.4f}'),
            'C_mixed': (pC, nin, f'offset30={s30 - s:+.4f}')}, c


# C glyph: hand-picked from the 3-box union (spine x[-.0483,-.0163] y+-.0515;
# top/bot bars x+-.0483 y [.0195,.0515]/[-.0515,-.0195]); centroid x=-0.01124.
C_CENTROID = np.array([-0.011240, 0.0])
C_CONTACTS = {
    'A_translation': (np.array([-0.0483, 0.0]), np.array([1.0, 0.0]),
                      'spine left edge, through centroid'),
    'B_rotation': (np.array([0.0450, 0.0515]), np.array([0.0, -1.0]),
                   'top bar top edge, far +x end (arm ~0.056)'),
    'C_mixed': (np.array([0.0043, 0.0515]), np.array([0.0, -1.0]),
                'top bar top edge, 30% offset (0.0155) from centroid'),
}


# ---------------- push-anything letter set (multi-convex VHACD, hydro) ------
ANYTHING = {  # letter -> (dir/name, body/link name)
    'I': 'I_shape_texture', 'C': 'C_shape_texture',
    'R': 'R_shape_texture', 'A': 'A_shape_video',
}


def load_obj_verts(path):
    return np.array([l.split()[1:4] for l in open(path) if l.startswith('v ')],
                    float)


def pick_contacts_cloud(verts):
    """A/B/C contacts from a mesh footprint: convex-hull boundary samples that
    are material-backed (within 2.5 mm of the projected vertex cloud), so
    concave mouths (C) are excluded. Same selection semantics as the hulls."""
    from scipy.spatial import ConvexHull
    P = verts[:, :2]
    c = P.mean(axis=0)
    hull = ConvexHull(P)
    V = P[hull.vertices]  # CCW
    maxhe = np.abs(P - c).max()
    samples = []  # (pt, inward_normal, tangent)
    n = len(V)
    for i in range(n):
        a, b = V[i], V[(i + 1) % n]
        L = np.linalg.norm(b - a)
        t = (b - a) / L
        nin = np.array([-t[1], t[0]])
        for s in np.arange(0.0, L + 1e-9, 0.002):
            p = a + min(s, L) * t
            if np.min(np.linalg.norm(P - p, axis=1)) < 0.0025:  # material-backed
                samples.append((p, nin, t))
    moms = np.array([p[0] * nin[1] - p[1] * nin[0] - (c[0] * nin[1] - c[1] * nin[0])
                     for (p, nin, t) in samples])
    offs = np.array([abs(np.dot(p - c, t)) for (p, nin, t) in samples])
    # A: through-centroid push on a LONG edge -> among low-moment samples,
    # prefer inward normal aligned with the footprint's minor axis
    cov = np.cov((P - c).T)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, np.argmax(evals)]
    lowmom = np.abs(moms) <= max(np.percentile(np.abs(moms), 15), 0.003)
    idxs = np.where(lowmom)[0]
    align = np.array([abs(np.dot(samples[i][1], major)) for i in idxs])
    iA = int(idxs[np.argmin(align + 10 * np.abs(moms[idxs]) / maxhe)])
    okB = offs >= 0.6 * maxhe
    iB = int(np.where(okB)[0][np.argmax(np.abs(moms[okB]))]) if okB.any() \
        else int(np.argmax(np.abs(moms)))
    iC = int(np.argmin(np.abs(np.abs(moms) - 0.3 * maxhe)))
    out = {}
    for name, i in [('A_translation', iA), ('B_rotation', iB), ('C_mixed', iC)]:
        p, nin, t = samples[i]
        out[name] = (p, nin, f'cloud mom={moms[i]:+.4f} off={offs[i]:.4f}')
    return out, c


def run_push(glyph, contact_pt, push_dir, object_set='prism'):
    if object_set == 'anything':
        nm = ANYTHING[glyph]
        sdf = os.path.join(URDF, nm, f'{nm}.sdf')
        v = load_obj_verts(os.path.join(URDF, nm, f'{nm}.obj'))
        spawn_z = -v[:, 2].min()
        zmid = spawn_z + 0.5 * (v[:, 2].min() + v[:, 2].max())
        body_name = nm
    else:
        sdf = os.path.join(URDF, f'push_{glyph.lower()}_glyph.sdf')
        spawn_z, zmid, body_name = ZMID, ZMID, 'c_glyph_base'
    builder = DiagramBuilder()
    plant, scene = AddMultibodyPlantSceneGraph(builder, DT)
    Parser(plant).AddModels(sdf)
    plant.RegisterCollisionGeometry(plant.world_body(), RigidTransform(),
                                    HalfSpace(), 'ground',
                                    CoulombFriction(GROUND_MU, GROUND_MU))
    mi = plant.AddModelInstance('pusher_model')
    pb = plant.AddRigidBody('pusher', mi,
                            SpatialInertia(0.057, np.zeros(3),
                                           UnitInertia.SolidSphere(PUSHER_R)))
    plant.RegisterCollisionGeometry(pb, RigidTransform(), Sphere(PUSHER_R),
                                    'pusher_col',
                                    CoulombFriction(PUSHER_MU, PUSHER_MU))
    plant.Finalize()
    diagram = builder.Build()
    sim = Simulator(diagram)
    ctx = sim.get_mutable_context()
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    body = plant.GetBodyByName(body_name)
    X0 = RigidTransform([0.0, 0.0, spawn_z])  # bottom on ground
    plant.SetFreeBodyPose(pctx, body, X0)
    d = np.array([push_dir[0], push_dir[1], 0.0])
    cp = np.array([contact_pt[0], contact_pt[1], zmid])
    p0 = cp - d * PUSHER_R  # sphere surface exactly touching the boundary pt
    plant.SetFreeBodyPose(pctx, pb, RigidTransform(p0))

    nsteps = int(DUR / DT)
    contact_steps = 0
    first_cp = None
    first_nhat = None
    obj_pt_local = None
    peak_fn = 0.0
    max_rp = 0.0
    pusher_bi = pb.index()
    obj_bi = body.index()
    port = plant.get_contact_results_output_port()
    for k in range(nsteps):
        t = k * DT
        plant.SetFreeBodyPose(pctx, pb, RigidTransform(p0 + d * SPEED * t))
        plant.SetFreeBodySpatialVelocity(pb, SpatialVelocity(np.zeros(3), d * SPEED), pctx)
        sim.AdvanceTo(t + DT)
        cr = port.Eval(pctx)
        hit = False
        for i in range(cr.num_point_pair_contacts()):
            info = cr.point_pair_contact_info(i)
            bA, bB = int(info.bodyA_index()), int(info.bodyB_index())
            if {bA, bB} == {int(pusher_bi), int(obj_bi)}:
                hit = True
                f = np.array(info.contact_force())
                nhat = np.array(info.point_pair().nhat_BA_W)
                fn = abs(np.dot(f, nhat))
                peak_fn = max(peak_fn, fn)
                if first_cp is None:
                    first_cp = np.array(info.contact_point())
                    first_nhat = nhat if bB == int(obj_bi) else -nhat
                    Xo = plant.GetFreeBodyPose(pctx, body)
                    obj_pt_local = np.array(Xo.inverse() @ first_cp)
        for i in range(cr.num_hydroelastic_contacts()):
            info = cr.hydroelastic_contact_info(i)
            surf = info.contact_surface()
            gA = plant.GetBodyFromFrameId(
                scene.model_inspector().GetFrameId(surf.id_M()))
            gB = plant.GetBodyFromFrameId(
                scene.model_inspector().GetFrameId(surf.id_N()))
            ids = {int(gA.index()), int(gB.index())}
            if ids == {int(pusher_bi), int(obj_bi)}:
                hit = True
                f = np.array(info.F_Ac_W().translational())
                peak_fn = max(peak_fn, float(np.linalg.norm(f)))
                if first_cp is None:
                    first_cp = np.array(surf.centroid())
                    fn_ = f / max(np.linalg.norm(f), 1e-12)
                    first_nhat = fn_ if int(gA.index()) == int(obj_bi) else -fn_
                    Xo = plant.GetFreeBodyPose(pctx, body)
                    obj_pt_local = np.array(Xo.inverse() @ first_cp)
        if hit:
            contact_steps += 1
        Xo = plant.GetFreeBodyPose(pctx, body)
        rpy = RollPitchYaw(Xo.rotation())
        max_rp = max(max_rp, abs(rpy.roll_angle()), abs(rpy.pitch_angle()))
    Xf = plant.GetFreeBodyPose(pctx, body)
    dxy = np.array(Xf.translation()[:2]) - np.array(X0.translation()[:2])
    dyaw = RollPitchYaw(Xf.rotation()).yaw_angle()
    travel = SPEED * DUR
    if obj_pt_local is not None:
        pt_f = np.array(Xf @ obj_pt_local)
        pt_0 = np.array(X0 @ obj_pt_local)
        obj_travel = float(np.dot((pt_f - pt_0)[:2], np.array(push_dir)))
    else:
        obj_travel = 0.0
    return dict(first_contact=first_cp, first_normal=first_nhat,
                contact_frac=contact_steps / nsteps, pusher_travel=travel,
                dx=dxy[0], dy=dxy[1], dyaw=dyaw, max_rp=max_rp,
                slip=travel - obj_travel, peak_fn=peak_fn)


def main():
    rows = []
    pred_rows = []
    for object_set in ['prism', 'anything']:
      for glyph in ['I', 'C', 'R', 'A']:
        if object_set == 'anything':
            nm = ANYTHING[glyph]
            verts = load_obj_verts(os.path.join(URDF, nm, f'{nm}.obj'))
            contacts, cen = pick_contacts_cloud(verts)
        elif glyph == 'C':
            contacts, cen = C_CONTACTS, C_CENTROID
        else:
            contacts, cen = pick_contacts(HULLS[glyph])
        for cname, (pt, nin, note) in contacts.items():
            reps = [run_push(glyph, pt, nin, object_set) for _ in range(3)]
            for r_i, r in enumerate(reps):
                fc = r['first_contact']
                fn = r['first_normal']
                rows.append([object_set, glyph, cname, r_i, note,
                             round(pt[0], 4), round(pt[1], 4),
                             round(nin[0], 3), round(nin[1], 3),
                             None if fc is None else round(fc[0], 4),
                             None if fc is None else round(fc[1], 4),
                             None if fn is None else round(fn[0], 3),
                             None if fn is None else round(fn[1], 3),
                             round(r['contact_frac'], 3), r['pusher_travel'],
                             round(r['dx'], 5), round(r['dy'], 5),
                             round(r['dyaw'], 4), round(r['max_rp'], 4),
                             round(r['slip'], 5), round(r['peak_fn'], 3)])
            r = reps[0]
            spread = max(abs(a['dx'] - r['dx']) + abs(a['dy'] - r['dy']) +
                         abs(a['dyaw'] - r['dyaw']) for a in reps)
            # prediction (quasi-static, object-agnostic point-contact analog)
            dvec = np.array([r['dx'], r['dy']])
            nrm = np.linalg.norm(dvec)
            cosv = float(np.dot(dvec, nin) / nrm) if nrm > 1e-9 else 0.0
            rarm = pt - cen
            mom = rarm[0] * nin[1] - rarm[1] * nin[0]
            # degenerate arm (<5 mm) -> expect ~no rotation; measured < 0.02 rad -> ~0
            exp_yaw = np.sign(mom) if abs(mom) > 0.005 else 0.0
            meas_yaw = np.sign(r['dyaw']) if abs(r['dyaw']) > 0.02 else 0.0
            pred_rows.append([object_set, glyph, cname,
                              round(nin[0], 3), round(nin[1], 3),
                              round(dvec[0] / nrm, 3) if nrm > 1e-9 else 0,
                              round(dvec[1] / nrm, 3) if nrm > 1e-9 else 0,
                              round(cosv, 3), int(exp_yaw), int(meas_yaw),
                              exp_yaw == meas_yaw,
                              round(r['dyaw'] / nrm, 3) if nrm > 1e-9 else '',
                              round(spread, 6)])
            print(f"{object_set} {glyph} {cname}: dxy=({r['dx']:.4f},{r['dy']:.4f}) "
                  f"dyaw={r['dyaw']:.4f} slip={r['slip']:.4f} "
                  f"cf={r['contact_frac']:.2f} maxrp={r['max_rp']:.3f} "
                  f"peakFn={r['peak_fn']:.2f} spread={spread:.2e}  [{note}]",
                  flush=True)
    with open(os.path.join(OUT, 'microtests/glyph_exact_contact.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['object_set', 'glyph', 'contact_type', 'rep', 'note',
                    'cmd_pt_x', 'cmd_pt_y', 'push_nx', 'push_ny',
                    'first_contact_x', 'first_contact_y',
                    'first_normal_x', 'first_normal_y',
                    'active_contact_frac', 'pusher_travel_m',
                    'dx_m', 'dy_m', 'dyaw_rad', 'max_rollpitch_rad',
                    'slip_m', 'peak_normal_force_N'])
        w.writerows(rows)
    with open(os.path.join(OUT, 'prediction/glyph_prediction_fidelity.csv'), 'w', newline='') as f:
        f.write('# quasi-static expectation only: the C3 model is object-agnostic '
                'point-contact, so dyaw_per_dxy at B substitutes for a C3-solve prediction\n')
        w = csv.writer(f)
        w.writerow(['object_set', 'glyph', 'contact', 'expected_dir_x', 'expected_dir_y',
                    'measured_dir_x', 'measured_dir_y', 'cos',
                    'expected_yaw_sign', 'measured_yaw_sign', 'match',
                    'dyaw_per_dxy', 'rep_spread'])
        w.writerows(pred_rows)
    print('CSV written.')


if __name__ == '__main__':
    main()
