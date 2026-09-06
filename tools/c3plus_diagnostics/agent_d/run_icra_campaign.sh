#!/usr/bin/env bash
set -uo pipefail
WT=/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-ycb-icra
R=$WT/results_icra_port/runs
run() { "$WT/tools/c3plus_diagnostics/agent_d/run_icra_draws.sh" "$@"; sleep 2; }
run anything_icra_c    0.5    -0.4    1.5708 600 "$R/v1" 7995
run anything_icra_c_v2 0.523  -0.4262 1.6692 600 "$R/v2" 7995
run anything_icra_c_v3 0.5247 -0.4041 1.2708 600 "$R/v3" 7995
run anything_icra_c_v4 0.5107 -0.4221 1.1996 600 "$R/v4" 7995
run anything_icra_c_v5 0.4845 -0.4286 1.163  600 "$R/v5" 7995
echo CAMPAIGN_DONE
