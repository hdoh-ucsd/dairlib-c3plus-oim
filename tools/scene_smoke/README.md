# Clean push-t runner

This repo has many demo-specific launchers under `examples/sampling_c3/`. For a single reproducible run in Docker or a standard checkout, use the generic scene-smoke runner instead.

## Run one push-t experiment

From the repo root:

```bash
bash tools/scene_smoke/run_push_t.sh /tmp/push_t_run 0.0 0.0 0.0
```

That command does the following:

1. Builds the three required binaries.
2. Runs the deterministic smoke experiment with `tools/scene_smoke/run_experiment.py`.
3. Renders the recorded trace to `push_t_render.mp4`.

The run output is stored under the chosen output directory, e.g. `/tmp/push_t_run`.

## Direct runner command

If you want the exact lower-level invocation without the helper wrapper:

```bash
python3 tools/scene_smoke/run_experiment.py \
  --scene open_task \
  --variant baseline \
  --start 1 \
  --goal 1 \
  --out /tmp/push_t_run
```

## Direct renderer command

```bash
python3 tools/scene_smoke/render_run_3d.py \
  --trace /tmp/push_t_run/state_trace.jsonl \
  --out /tmp/push_t_run/push_t_render.mp4 \
  --object-sdf examples/sampling_c3/urdf/push_t_oimscale_m01.sdf \
  --goal 0.0 0.0 0.0 \
  --title push_t \
  --fps 10 \
  --max-frames 1200
```

This is the clean, repeatable pattern for Docker runs: run the generic smoke binary stack once, then render the saved trace from the result directory.
