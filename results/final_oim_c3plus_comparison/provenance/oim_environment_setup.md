# OIM MJX local environment setup + smoke test (2026-09-06)

Repo (read-only source): `/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX`
Venv: `/root/oim_mjx_venv` (python 3.12.9, miniconda base `python3.12 -m venv`)
GPU: RTX 5070 Ti, driver 580.65.05 / CUDA 13.0, WSL2.

## Install recipe that WORKS (hand-pip; the uv/cuda13 path does NOT on this box)

```bash
/root/miniconda3/bin/python3.12 -m venv /root/oim_mjx_venv
V=/root/oim_mjx_venv/bin/pip
$V install -U pip
$V install "mujoco==3.8.0" "mujoco-mjx==3.8.0" "jax[cuda12]==0.8.3" \
           "flax==0.12.4" "evosax==0.2.0" "huggingface_hub>=0.29.3" \
           "interpax>=0.3.12" "matplotlib>=3.10.0" pyyaml pytest \
           "warp-lang>=1.12.1,<1.13.0"
$V install -e /root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX --no-deps
```

Versions mirror `uv.lock` (mujoco/mjx 3.8.0, jax 0.8.3, warp-lang 1.12.1, flax 0.12.4,
evosax 0.2.0) EXCEPT the CUDA flavor: cuda12 wheels instead of the locked cuda13.
`pip check` complains only about ruff/pre-commit (dev tools, deliberately skipped).

### Two machine-specific traps (both root-caused)

1. **jax[cuda13] wheels segfault on this WSL2 box** — SIGSEGV inside
   `xla_client.make_c_api_client` (PJRT CUDA client creation), reproduced at
   plugin versions 0.10.2 and 0.11.1. `jax[cuda12]` works on the same driver.
   Therefore `uv sync --frozen` (which installs the locked jax[cuda13]) would
   reproduce the segfault — do not use the lock's CUDA flavor here.
2. **Every run needs `LD_LIBRARY_PATH=/usr/lib/wsl/lib`.** A stale stub
   `/lib/x86_64-linux-gnu/libcuda.so` shadows the real WSL driver; Warp then
   fails `cuGetProcAddress` and reports 0 CUDA devices
   (`IndexError: runtime.cuda_devices[ordinal]` inside mjx-warp FFI). With the
   env var Warp sees `cuda:0 sm_120` and everything runs.

Also: mujoco 3.12 (what unpinned resolution gives) is incompatible with
warp-lang 1.12 (`kernel() got an unexpected keyword argument 'grid_stride'`) —
stay at the locked mujoco 3.8.0.

Verification: `jax.devices()` -> `[CudaDevice(id=0)]`; 512x512 GPU matmul OK;
`from mujoco import mjx` OK; `import oim` OK.

## Running a benchmark trial

Entry point = one script per scene under `examples/pusht/`, all sharing
`oim/experiment.py`'s CLI. The OIM method is the `admm` subcommand:

```bash
cd /root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX
LD_LIBRARY_PATH=/usr/lib/wsl/lib /root/oim_mjx_venv/bin/python \
    examples/pusht/open_table.py --robot xarm6 admm --headless [--steps N] [--seed S] [--start K --goal K]
```

- **Scene selection**: pick the script — `open_table.py`, `single_obstacle.py`,
  `shelf_gap.py`, `ycb_clutter.py`, `icra_sign.py`, `clutter.py` (scene names are
  keys of `oim/utils/scenes.SCENES`; the script declares `Experiment(world="3d", scene=...)`).
- **Embodiment**: `--robot xarm6` (config `oim/configs/robots/xarm6.yaml`; `point`
  is the other embodiment; `xarm6_real.yaml` is the hardware variant).
- **Algorithm**: subcommands `admm` (OIM), `mppi`, `ps`, `c3` (their C3+ reimplementation)
  — flat baselines take `--iterations`, admm takes `--n-admm/--rho/--gamma/--plant` etc.
- **Trial seed / pose draw**: `--seed` (default 0 from `run.seed`) redraws planner
  noise AND the start/goal draw; `--start K --goal K` pin pose keys from
  `examples/poses/<scene>.yaml` (5 starts x 5 goals per scene, keys "1".."5";
  unset = drawn from the seed's rng, recorded in the run file as
  `start_index`/`goal_index` so random runs are reproducible).
- **Time cap**: `--steps` (xarm6 default `run.steps: 2000`) at control dt 0.05 s;
  the loop **stops early** on success (`pos_err < 0.05 m` and `theta_err < 0.1 rad`,
  `run.goal_pos_tol`/`goal_theta_tol`; re-scorable offline via
  `oim/run_eval.py --pos-tol/--theta-tol`).
- **Other knobs**: `--samples` (256), `--horizon` (32; admm consensus H 24 per
  comment), `--plant analytic|mujoco` (xarm6 default mujoco), `--warp/--no-warp`
  (xarm6 default warp: MuJoCo-Warp rollouts), `--record` (xarm6 `run.record: true`
  — mp4 always saved; no `--no-record` flag), `--gamma0-deg`, `--object-samples`.
- Sweeps: `oim/run_launch.py` + `oim/configs/sweeps/*.yaml`; scoring:
  `python -m oim.run_eval` (grades saved run files, no re-run).

### Outputs (per run, shared timestamped stem `xarm6_<scene>_admm_wrench_<ts>`)

- Run file: `<repo>/oim/results/runs/*.json` (states, wrenches, plans, residuals,
  hyperparameters, `reached` flag) — what `run_eval.py` globs.
- Video: `<repo>/oim/recordings/*.mp4`; summary plot: `<repo>/oim/recordings/*.png`.

## Smoke test result (PASSED end-to-end)

Command: `... open_table.py --robot xarm6 admm --headless --steps 10` (seed 0,
drawn start "5" / goal "4", backend warp, plant mujoco, 256 samples, H 32, n_admm 4).
Ran 10 control steps, printed per-step pos_err/theta_err/primal/dual, saved
`oim/results/runs/xarm6_open_table_admm_wrench_20260906_190406.json` + mp4 + png.
First-run compile time is dominated by Warp module compilation (worst single
module `ccd_kernel` 12.7 s; cached afterwards under `~/.cache/warp/1.12.1`) plus
JAX JIT.

## Wall-time (measured, warm Warp kernel cache)

60-step xarm6 open_table admm run (`--steps 60`, seed 0): total wall 116.3 s
(`/usr/bin/time`). Run files log per-step `compute_time`:
first step 16.1 s (JAX JIT + Warp graph capture), thereafter mean 1.20 s/step,
median 0.97 s/step; the rest of the wall is ~25 s process startup/model build +
video/plot writing. Cold first-ever process additionally pays Warp module
compilation (~30-60 s total, worst module `ccd_kernel` 12.7 s; cached in
`~/.cache/warp/1.12.1`).

Benchmark estimate at default `--steps 2000` (early-stop on success):
~35 s fixed overhead + ~1.2 s/step -> a censored (never-succeeds) trial is
~40 min; a trial reaching the goal at step N costs ~35 s + 1.2*N s (e.g.
success at step 300 ~ 6.5 min). single_obstacle/shelf_gap use the same xarm6
config and per-step cost should be comparable (same model size + one obstacle
body); expect the same ~1.2 s/step order.
