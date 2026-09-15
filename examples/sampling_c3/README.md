# Sampling C3+ native components

Native simulation and controller code for the xArm6 scene benchmark. See the
[root README](../../README.md) for Docker setup, builds, campaign commands, and
result handling.

| Component | Purpose |
| --- | --- |
| `franka_sim.cc` | Drake simulation and state publication |
| `franka_osc_controller.cc` | Low-level tracking of planned end-effector trajectories |
| `franka_sampling_c3_controller.cc` | Sampling C3+ planner and optional fixed-goal yaw override |
| `goal_generator.*`, `generate_samples.*`, `reposition.*`, `sampling_c3_utils.*` | Shared native algorithms and model construction |
| `parameter_headers/` | Typed YAML configuration schemas |
| `shared_parameters/` | Shared settings, controller profiles, scene geometry, starts, and goals |
| `urdf/` | Robot, tool, object, and obstacle models and meshes |

The [experiment catalogue](shared_parameters/experiments.yaml) separates start
positions, goal positions, and orientations, with shared object profiles and
scene overrides. The runner composes each trial's native YAMLs in its output
directory and passes `--controller_params` to the native processes. The 150
matched demo names remain identifiers without individual directories. Three
`push_t_bt010_*_xarm6` configuration directories remain for native compatibility;
omitting `--controller_params` retains the legacy `--demo_name` file lookup.

The copied Procman launch files targeted retired demos and have been removed.
Use `python3 -m c3plus.utils` for maintained experiment launches. Optional
native visualization, hardware bridge code, and binary-LCM logging/analysis
utilities remain separate from this workflow.

## Method and citation

Based on *Approximating Global Contact-Implicit MPC via Sampling and Local
Complementarity*.

[Project](https://approximating-global-ci-mpc.github.io/) ·
[Paper](https://arxiv.org/abs/2505.13350) ·
[Supplemental video](https://youtu.be/rv9n8Uyvoh0)

```
@article{venkatesh2025approximating,
 title={Approximating Global Contact-Implicit MPC via Sampling and Local Complementarity},
 author={Sharanya Venkatesh* and Bibit Bianchini* and Alp Aydinoglu and William Yang and Michael Posa},
 year={2025},
 journal={arXiv preprint arXiv:2505.13350},
 website={https://approximating-global-ci-mpc.github.io/}
}
```
