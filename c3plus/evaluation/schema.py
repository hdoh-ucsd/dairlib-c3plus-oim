"""Ordered result-schema definitions; no controller or runtime defaults."""

BLOCKS = ["goal_pos", "goal_theta", "obstacle", "support", "approach",
          "align", "tilt", "tip_z_cost", "contact_z", "pusher_obstacle",
          "robot_contact", "effort", "admm_penalty"]

SNAPSHOT_SEMANTICS_VERSION = 3
EXECUTION_SEMANTICS_VERSION = 4
SNAPSHOT_STATE_ARRAYS = ("time", "object_pose", "object_velocity", "robot_pos", "robot_vel", "qpos", "qvel",
              "control_step", "object_pose_3d", "robot_joint_effort", "tip_z_state", "tip_tilt_state",
              "position_error_m", "orientation_error_rad", "physical_contact_active",
              "pusher_object_gap", "min_obstacle_clearance", "evaluation_total")
SNAPSHOT_INTERVAL_ARRAYS = ("robot_control", "tip_z", "tip_tilt",
                 "contact_normal_force_z", "robot_contact_force")


def projection_schema():
    return {
        "version": "c3plus-reference-projection-v1",
        "indexing": "State arrays have steps_run+1 entries. steps_run is the number of adjacent "
                    "recorded intervals, not executed control actions; n_control_steps counts snapshots. "
                    "Entry 0 is the first observed state, not the initial condition. Interval arrays have "
                    "steps_run entries; tip_z and tip_tilt use state[i+1]. No applied transition control is recorded.",
        "frames": "object_pose=[x,y,theta] and robot_pos=tip [x,y] in world coordinates; "
                  "object_footprint_body is in object coordinates. object_pose_3d=[qw,qx,qy,qz,x,y,z].",
        "units": {"time": "s (simulation)", "compute_time": "s (wall)", "position": "m",
                  "orientation": "rad", "linear_velocity": "m/s", "angular_velocity": "rad/s",
                  "robot_joint_effort": "N m", "contact_force": "N"},
        "sampling": "Asynchronous latest-message snapshots on C3_DEBUG_CURR. Object/robot timestamps "
                    "were not retained individually. Times are preserved, without resampling or a synthetic t=0. "
                    "control_dt is the mean observed state interval when all intervals are positive; use dynamic.time for execution time.",
        "velocities": "object_velocity and robot_vel are backward finite-difference estimates at actual "
                      "sample times (yaw differences wrapped to [-pi,pi)). First estimates and nonpositive-dt "
                      "intervals are null. Repeated cached poses may produce zero despite physical motion.",
        "qpos": "Projected layout in static.state_layout: recorded robot joints, then object [x,y,yaw,z]. "
                "qvel combines recorded robot joint velocities with estimated object [vx,vy,omega,vz]. "
                "These are not Drake's complete generalized coordinates; object_pose_3d preserves quaternion pose.",
        "controls": "robot_control contains null vectors: applied transition controls were not recorded. "
                    "robot_joint_effort preserves published efforts at each snapshot, without action alignment.",
        "plans": "Predicted trajectories and wrench/consensus series were not recorded and are omitted.",
        "missing": {"robot_control": "No aligned control input recording",
                    "compute_time": "No per-step optimization timing recording",
                    "contact_normal_force_z": "No measured contact force recording",
                    "robot_contact_force": "No measured contact force recording",
                    "object_limit_surface_d": "No equivalent native C3+ limit surface parameters",
                    "object_wrench_limit": "No equivalent native C3+ planar wrench limit"},
        "evaluation": "evaluation_costs and diagnostic arrays are state-aligned CSV values; evaluation_total "
                      "sums available terms only. Null denotes unavailable data, including CSV NaN. "
                      "physical_contact_active is a planar proximity flag, not measured contact. "
                      "Legacy success means ever meeting both tolerances, not final-state success.",
    }
