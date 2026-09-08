#!/usr/bin/env bash
# 30 s cadence load/memory sampler for the compute-utilization report.
OUT="${1:?}"
while true; do
  echo "$(date -Is) load=$(cut -d' ' -f1-3 /proc/loadavg) mem_used_mb=$(free -m | awk '/Mem/{print $3}')" >> "$OUT"
  sleep 30
done
