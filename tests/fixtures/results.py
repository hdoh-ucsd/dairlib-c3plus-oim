from contextlib import redirect_stdout
import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from unittest.mock import patch
import yaml

from c3plus import configs as S
from c3plus.utils import run as R
from c3plus.visualization import objects as V
from c3plus.evaluation import exporter as P, package as A

class WorkflowFixtures:
    def source_state_fixture(self):
        empty = {"encoding": "utf-8", "content": "", "size_bytes": 0,
                 "sha256": hashlib.sha256(b"").hexdigest()}
        return {"format": "git-source-state/v1", "base_commit": "test-commit",
                "worktree_dirty": False, "scope": "test fixture",
                "tracked_patch": empty, "git_status": dict(empty), "untracked_files": {}}


    def copy_demo_configs(self, demo, destination):
        for source in S.load_demo_configs(demo):
            target = destination / source.relative_to(S.REPO)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)


    def args(self, **changes):
        values = dict(manifest=None, obstacle_cost="both", scenes=list(R.SCENES), pairs="all")
        values.update(changes)
        return argparse.Namespace(**values)


    def renderer(self):
        try:
            from c3plus.visualization import render as render_run_3d
            return render_run_3d
        except ModuleNotFoundError as exc:
            self.skipTest(f"Renderer dependencies unavailable: {exc}")


class MeshPreviewFixtures:
    def settings(self, *flags):
        stream = io.StringIO()
        with patch.object(V, "render_image") as viewer, redirect_stdout(stream):
            V.main([*flags, "--dry-run"])
        viewer.assert_not_called()
        return json.loads(stream.getvalue())


class ObjectFixtures:
    def dry_run(self, out, *flags):
        stream = io.StringIO()
        with redirect_stdout(stream), patch.object(R, "run_one") as launch:
            R.main(["--scene", "open_task", "--out", str(out), "--dry-run", *flags])
        launch.assert_not_called()
        return json.loads(stream.getvalue())


class ProjectionFixtures:
    def fixture(self):
        run_id = "synthetic_banana_run"
        channel = "OBJECT_banana_base_STATE_SIMULATION"
        cfg = {"object_name": "banana", "object_body_name": "banana_base",
               "object_channel_substring": "banana_base", "goal": [.4, -.4, 0],
               "footprint": [[-.02, -.02], [.02, -.02], [.02, .02], [-.02, .02]],
               "pusher_radius": .00555, "block_half_height": .01837,
               "tip_target_z": -.012, "tip_floor_z_real": -.02345,
               "obstacles": {}}
        times = [10.0, 10.1, 10.4, 10.4, 10.2]
        xs, ys = [.30, .31, .37, .40, .42], [.10, .12, .09, .09, .08]
        zs = [-.029, -.027, -.021, -.019, -.020]
        yaws = [math.pi - .05, -math.pi + .05, -math.pi + .11,
                -math.pi + .20, -math.pi + .30]
        tip_xs, tip_ys = [.20, .21, .27, .29, .30], [.10, .095, .11, .10, .10]
        steps, rows = [], []
        for index, (time, x, y, z, yaw) in enumerate(zip(times, xs, ys, zs, yaws)):
            step = 10 + 3 * index  # A logged sample gap is not a fabricated interval.
            steps.append({"control_step": step, "sim_time": time,
                          "robot_q": [index + j / 10 for j in range(5)],
                          "robot_v": [j / 100 for j in range(5)],
                          "robot_u": [100 + index + j for j in range(5)],
                          "objects": {channel: [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2), x, y, z]}})
            rows.append({"run_id": run_id, "scenario": "open_task", "control_step": step,
                         "sim_time": time, "object_x": x, "object_y": y, "object_yaw": yaw,
                         "goal_x": .4, "goal_y": -.4, "goal_yaw": 0,
                         "position_error_m": math.hypot(x - .4, y + .4),
                         "orientation_error_rad": abs(P.wrap(yaw)),
                         "tip_x": tip_xs[index], "tip_y": tip_ys[index],
                         "tip_z": -.012 + .001 * index,
                         "tip_roll": math.pi - .1 * index, "tip_pitch": 0, "tip_yaw": 0,
                         "physical_contact_active": int(index % 2 == 0),
                         "pusher_object_gap": .002, "min_obstacle_clearance": float("nan"),
                         **{key: float("nan") if key == "admm_penalty" else 1.0 for key in P.BLOCKS},
                         "evaluation_total": 12.0})
        summary = {"run_id": run_id, "scenario": "open_task", "success": False,
                   "t_success": None, "first_success_t": None,
                   "final_position_error": rows[-1]["position_error_m"],
                   "final_orientation_error": rows[-1]["orientation_error_rad"],
                   "best_pos_err": .125, "best_ang_err": .25,
                   "n_control_steps": len(rows), "sim_time_end": times[-1],
                   "object_name": "banana", "legacy_annotation": {"keep": "unchanged"}}
        return cfg, summary, steps, rows


    def project(self, directory, cfg, summary, steps, rows, **kwargs):
        return P.project_result(Path(directory), "open_task", summary["run_id"],
                                cfg, summary, steps, rows, **kwargs)


    def physical_fixture(self):
        updates = [{"update": i, "utime": 100_000+i, "mode": "c3" if i else "reposition"}
                   for i in range(4)]
        states = [{"boundary_step": i, "plan_utime": 100_000 if i == 0 else 100_002,
                   "sim_time": sim, "wall_time": wall,
                   "objects": [{"q": [1, 0, 0, 0, .2+.1*i, -.4, -.02], "v": [0]*6}],
                   "robot_q": [0]*5, "robot_v": [0]*5}
                  for i, (sim, wall) in enumerate(zip([7.1, 7.2, 7.35], [0.0, .035, .08]))]
        states[-1]["reason"] = "step_budget"
        native = {"headers": [{"alignment": "physical_policy_boundaries_v1", "step_budget": 2,
                               "object_channels": ["OBJECT_banana_base_STATE_SIMULATION"]}],
                  "boundaries": states[:2], "terminals": states[2:]}
        return native, updates


    def semantic_result(self):
        cfg, summary, steps, rows = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows)
        return P._json_values(result)


RUN_ID = "exponential_open_task_banana_s02g02_seed42"


VIDEO_PROBE = {
    "streams": [{"codec_type": "video", "codec_name": "h264", "width": 960,
                 "height": 720, "nb_frames": "3", "duration": "0.3"}],
    "format": {"duration": "0.3", "format_name": "mov,mp4,m4a,3gp,3g2,mj2"},
}


def file_snapshot(root):
    """Include link targets and empty directories without following links."""
    values = {}
    for path in sorted(root.rglob("*")):
        key = str(path.relative_to(root))
        if path.is_symlink():
            values[key] = ("symlink", os.readlink(path))
        elif path.is_dir():
            values[key] = ("directory", None)
        else:
            values[key] = ("file", path.read_bytes())
    return values


class ArtifactFixtures:
    def make_run(self, root, success=False):
        root.mkdir(parents=True, exist_ok=True)
        cfg = {"object_name": "banana", "object_body_name": "banana_base",
               "object_channel_substring": "banana_base", "goal": [.4, -.4, 0],
               "footprint": [[-.02, -.02], [.02, -.02], [.02, .02], [-.02, .02]],
               "pusher_radius": .00555, "block_half_height": .01837,
               "tip_target_z": -.01063, "tip_floor_z_real": -.02345, "obstacles": {}}
        steps, rows, trace = [], [], []
        for index, (time, x) in enumerate(zip((.1, .2, .3), (.30, .32, .34)), 1):
            pose = [1, 0, 0, 0, x, -.4, -.029]
            q = [index + j / 10 for j in range(5)]
            step = {"control_step": index, "sim_time": time, "robot_q": q,
                    "robot_v": [.1 * j for j in range(5)],
                    "robot_u": [index * 10 + j for j in range(5)],
                    "objects": {"OBJECT_banana_base_STATE_SIMULATION": pose},
                    "additional_recorded_field": {"sequence": index, "value": "preserve me"}}
            steps.append(step)
            trace.append({"t": time, "q": q, "obj": pose, "pos_err": .4 - x,
                          "ang_err": 0, "extra_trace_value": index})
            rows.append({"run_id": RUN_ID, "scenario": "open_task", "control_step": index,
                         "sim_time": time, "object_x": x, "object_y": -.4, "object_yaw": 0,
                         "goal_x": .4, "goal_y": -.4, "goal_yaw": 0,
                         "position_error_m": .4 - x, "orientation_error_rad": 0,
                         "tip_x": x - .03, "tip_y": -.4, "tip_z": -.01063,
                         "tip_roll": math.pi, "tip_pitch": 0, "tip_yaw": 0,
                         "physical_contact_active": 0, "pusher_object_gap": .02,
                         "min_obstacle_clearance": float("nan"), "evaluation_total": 3.5,
                         **{name: float("nan") if name == "admm_penalty" else .25
                            for name in P.BLOCKS}})
        for filename, records in (("steps_raw.jsonl", steps), ("state_trace.jsonl", trace)):
            (root / filename).write_text("".join(json.dumps(row) + "\n" for row in records))
        with (root / f"{RUN_ID}_metrics.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (root / "evaluation_scene_config.yaml").write_text(yaml.safe_dump(cfg))
        config = root / "config"
        config.mkdir()
        (config / "controller.yaml").write_text(
            "# Preserve the actual run configuration, including comments.\n"
            "object_body_name: banana_base\n"
            "sampling_params_file: config/sampling.yaml\n")
        (config / "sampling.yaml").write_text("sample_random_seed: 42\nsampling_strategy: 8\n")
        (config / "simulation.yaml").write_text("dt: 0.001\n")
        (config / "goal.yaml").write_text("goal_position: [0.4, -0.4]\n"
                                         "position_success_threshold: 0.02\n"
                                         "orientation_success_threshold: 0.1\n")
        status = {"run_id": RUN_ID, "scene": "open_task", "object_name": "banana",
                  "wrapper_rc": 0, "postprocess_rc": 0, "render_rc": 0, "cost_fig_rc": 0,
                  "seed": 42, "seed_verified": True, "failures": [], "success": success,
                  "simulation_wall_seconds": 12.5,
                  "config_sha256": {str(path.relative_to(root)):
                                    hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in config.glob("*.yaml")},
                  "diagnostic_extension": {"native_timeout": False, "stop_reason": "wall_cap"}}
        (root / "runtime_status.json").write_text(json.dumps(status, indent=2) + "\n")
        costs = {"W_Z_TIP": 2.5, "diagnostic_only": True}
        manifest = {"run_id": RUN_ID, "scenario": "open_task", "goal": cfg["goal"],
                    "tolerances": {"pos_tol": .05, "ang_tol": .1},
                    "evaluation": {"costs": costs}, "git_commit": "fixture-commit"}
        (root / f"{RUN_ID}_manifest.yaml").write_text(yaml.safe_dump(manifest))
        summary = {"run_id": RUN_ID, "scenario": "open_task", "success": success,
                   "n_control_steps": len(steps), "sim_time_end": .3,
                   "first_success_t": None, "final_position_error": .06,
                   "final_orientation_error": 0, "legacy_annotation": {"keep": "unchanged"}}
        result = P.project_result(root, "open_task", RUN_ID, cfg, summary, steps, rows,
                                  evaluation_costs=costs)
        result_path = root / f"{RUN_ID}_result.json"
        P.write_result_json(result_path, result)
        recorder_final = {"steps": 3, "success": success, "custom_counter": 17}
        for phase in ("launcher", "planner", "sim", "osc", "postprocess", "render", "cost_fig"):
            (root / f"{phase}.log").write_text(f"[COMMAND] fixture {phase}\n{phase}: λ diagnostic\n")
        (root / "planner.log").write_text(
            "[COMMAND] fixture planner\n[CWD] /recorded/checkout\n"
            "WARNING preserve an early diagnostic\n"
            + "".join(f"ordinary progress {index}\n" for index in range(75)))
        (root / "recorder.log").write_text("Recorder ready\nFINAL " + json.dumps(recorder_final) + "\n")
        scratch = root / "tmp/matplotlib"
        scratch.mkdir(parents=True)
        (scratch / "fontlist.json").write_text('{"generated_cache": true}\n')
        (root / f"{RUN_ID}_cost_diagnostics.png").write_bytes(b"derived diagnostic figure")
        # ffprobe is the one external boundary mocked below; no encoder is run.
        video = b"\x00\x00\x00\x18ftypmp42\x00\x00\x00\x00mp42isom" + b"fixture video payload"
        (root / f"{RUN_ID}.mp4").write_bytes(video)
        (root / "RUN_COMPLETE").touch()
        return {"result": json.loads(result_path.read_text()), "steps": steps, "trace": trace,
                "status": status, "cfg": cfg, "manifest": manifest, "recorder_final": recorder_final,
                "video": video, "before": file_snapshot(root)}


    def ffprobe(self, argv, *args, **kwargs):
        self.assertEqual(Path(argv[0]).name, "ffprobe")
        return subprocess.CompletedProcess(argv, 0, json.dumps(VIDEO_PROBE), "")


    def compact(self, root, **kwargs):
        with patch.object(A.subprocess, "run", side_effect=self.ffprobe):
            return A.compact_run(root, RUN_ID, **kwargs)


    def assert_rejected_without_changes(self, root, **kwargs):
        before = file_snapshot(root)
        with self.assertRaises((ValueError, RuntimeError, OSError)):
            self.compact(root, **kwargs)
        self.assertEqual(file_snapshot(root), before)


    def interrupt_cleanup(self, root):
        original_unlink = Path.unlink
        target = root / "steps_raw.jsonl"

        def injected_disk_error(path, *args, **kwargs):
            if path == target:
                raise PermissionError("Injected cleanup failure")
            return original_unlink(path, *args, **kwargs)

        with patch.object(Path, "unlink", autospec=True, side_effect=injected_disk_error), \
                self.assertRaisesRegex(PermissionError, "Injected cleanup failure"):
            self.compact(root)


def launch_subprocess(command, **kwargs):
    """Invoke the copied runtime module in the fake checkout used by launch tests."""
    if command[0] == "bash":
        path = Path(command[1])
        kwargs["cwd"] = path.parents[2]
        command = [sys.executable, "-m", "c3plus.runtime.launcher", *command[2:]]
    return subprocess.run(command, **kwargs)
