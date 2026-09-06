"""Shared plant / executor / data loading for the xArm6 kinematic-trap analysis."""
import json, re, os
import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import JacobianWrtVariable

WORKTREE = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
DATA_ROOT = "/root/push_anything_ADMM/results"
OUT = os.path.join(WORKTREE, "results/panda_to_xarm6_final/trap_analysis")

KC, ALPHA, LAM, QDOT_MAX = 4.0, 2.0, 0.05, 0.5
W = np.diag([1.0, 1.0, 1.0, 0.2, 0.2])


def build_plant():
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    parser.AddModels(os.path.join(WORKTREE, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
    parser.AddModels(os.path.join(WORKTREE, "examples/sampling_c3/urdf/end_effector_full.urdf"))
    X = RigidTransform(RollPitchYaw(3.1415, 0.0, 0.0), [0, 0, 0.107])
    plant.WeldFrames(plant.GetFrameByName("xarm6_link6"),
                     plant.GetFrameByName("end_effector_flange"), X)
    plant.Finalize()
    assert plant.num_positions() == 5, plant.num_positions()
    ctx = plant.CreateDefaultContext()
    return plant, ctx


class Exec:
    def __init__(self):
        self.plant, self.ctx = build_plant()
        self.tip = self.plant.GetBodyByName("end_effector_tip")
        self.world = self.plant.world_frame()

    def fk(self, q):
        self.plant.SetPositions(self.ctx, np.asarray(q))
        X = self.plant.EvalBodyPoseInWorld(self.ctx, self.tip)
        return X.translation(), X.rotation().matrix()

    def jac(self, q):
        """5x5 J rows [vx,vy,vz,wx,wy]. Drake spatial = [w(3); v(3)]."""
        self.plant.SetPositions(self.ctx, np.asarray(q))
        Js = self.plant.CalcJacobianSpatialVelocity(
            self.ctx, JacobianWrtVariable.kV, self.tip.body_frame(),
            np.zeros(3), self.world, self.world)  # 6x5
        return np.vstack([Js[3:6, :], Js[0:2, :]])

    def v_task(self, q, p_des):
        p_tip, R = self.fk(q)
        a = R @ np.array([0.0, 0.0, 1.0])
        s = np.sign(a[2]) if a[2] != 0 else 1.0
        v = np.zeros(5)
        v[0:3] = KC * (np.asarray(p_des) - p_tip)
        v[3] = s * ALPHA * a[1]
        v[4] = -s * ALPHA * a[0]
        return v, p_tip, R

    @staticmethod
    def dls(J, v, lam=LAM, Wm=W):
        Jw = Wm @ J
        qdot = np.linalg.solve(Jw.T @ Jw + lam**2 * np.eye(J.shape[1]), Jw.T @ Wm @ v)
        m = np.max(np.abs(qdot))
        fac = min(1.0, QDOT_MAX / m) if m > 0 else 1.0
        return qdot, qdot * fac, fac

    def step(self, q, p_des, lam=LAM, Wm=W):
        v, p_tip, R = self.v_task(q, p_des)
        J = self.jac(q)
        raw, sat, fac = self.dls(J, v, lam, Wm)
        return dict(v=v, p_tip=p_tip, R=R, J=J, raw=raw, sat=sat, fac=fac)


OSC_RE = re.compile(r"XARM6_5J t=(\S+) p_des=\s*(\S+)\s+(\S+)\s+(\S+) p_tip=\s*(\S+)\s+(\S+)\s+(\S+) \|qdot_cmd\|=(\S+)")


def load_trial(run, trial):
    d = f"{DATA_ROOT}/panda_to_xarm6_port_final_{run}/xarm6_trial{trial}"
    rows = []
    with open(d + "/osc.log") as f:
        for line in f:
            m = OSC_RE.search(line)
            if m:
                g = [float(x) for x in m.groups()]
                rows.append(dict(t=g[0], p_des=np.array(g[1:4]), p_tip=np.array(g[4:7]), qn=g[7]))
    states = []
    with open(d + "/state_trace.jsonl") as f:
        for line in f:
            j = json.loads(line)
            states.append((j["t"], np.array(j["q"])))
    st = np.array([s[0] for s in states])
    sq = np.stack([s[1] for s in states])
    return rows, st, sq


def q_at(st, sq, t):
    i = np.argmin(np.abs(st - t))
    return sq[i], st[i]


TRIALS = [(r, i) for r in ("r4", "r5") for i in range(1, 6)]
