#!/usr/bin/env bash
# Package every completed (RUN DONE) grid run of one variant that is not yet
# packaged (no *_result.json). Runs under nice so live sims keep priority.
# Usage: package_all_done.sh VARIANT [JOBS]
set -uo pipefail
V="${1:?variant}"; J="${2:-2}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
R="$WT/results/c3plus_relu_chomp_comparison/$V"
list=()
for d in "$R"/*/s*g*/; do
  [ -L "${d%/}" ] && continue
  grep -q '^RUN DONE' "$d/launcher.log" 2>/dev/null || continue
  ls "$d"/*_result.json >/dev/null 2>&1 && ls "$d"/*.mp4 >/dev/null 2>&1 \
    && ls "$d"/*_cost_diagnostics.png >/dev/null 2>&1 && continue
  scene=$(basename "$(dirname "$d")"); cell=$(basename "$d")
  list+=("$V $scene $cell")
done
echo "${#list[@]} runs to package (variant=$V, jobs=$J)"
printf '%s\n' "${list[@]}" | nice -n 19 xargs -P "$J" -L 1 bash "$WT/tools/relu_chomp/package_grid_run.sh"
echo "PACKAGING PASS DONE ($V)"
