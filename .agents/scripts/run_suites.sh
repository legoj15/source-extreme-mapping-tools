#!/usr/bin/env bash
# Run the quick regression suites against the binaries in game/bin/x64 and
# print one pass/fail line per suite. Meant for after a merge or a rebuild.
#
# Usage (from anywhere inside this repository):
#   bash .agents/scripts/run_suites.sh [--only name,name] [--list]
#
# Suites (each passes its own default -game; see AGENTS.md):
#   bspwalk_selftest  bspwalk.exe -selftest
#   bspwalk_fixtures  unit_test/bspwalk/run_bspwalk_tests.py --walker-args=-vischeck  (about 2 min)
#   bspwalk_cache     unit_test/bspwalk/run_bspwalk_cache_tests.py: -eyeverify on every fixture BSP,
#                     and the same tiers at any eye cache size and thread count (needs the
#                     fixture BSPs bspwalk_fixtures compiles)
#   lmoptimizer       pytest unit_test/lmoptimizer
#   proplm            pytest unit_test/proplm
#   vvis_sightline    unit_test/vvis_sightline/run_sightline_tests.py
#   vvis_mergeflow    unit_test/vvis_mergeflow/run_mergeflow_tests.py
#
# Not run here (slow, or need the SDK reference binaries): run_vvis_tests.ps1,
# run_vrad_tests.ps1, run_bspwalk_tests.py --regen.
# Logs: game/bin/x64/unit_test/unit_test_logs/<timestamp>_suites/<suite>.log
# Exit code: 0 when every selected suite passed, 1 otherwise.
set -uo pipefail

root=$(git rev-parse --show-toplevel)
x64="$root/game/bin/x64"
ut="$x64/unit_test"
all="bspwalk_selftest bspwalk_fixtures bspwalk_cache lmoptimizer proplm vvis_sightline vvis_mergeflow"
only=$all

while [ "$#" -gt 0 ]; do
  case "$1" in
    --only) only=$(echo "$2" | tr ',' ' '); shift 2 ;;
    --list) echo "$all" | tr ' ' '\n'; exit 0 ;;
    *) sed -n '2,22p' "$0" >&2; exit 2 ;;
  esac
done

logdir="$ut/unit_test_logs/$(date +%Y%m%d_%H%M%S)_suites"
mkdir -p "$logdir"

run_suite() {
  case "$1" in
    bspwalk_selftest) (cd "$x64" && ./bspwalk.exe -selftest) ;;
    bspwalk_fixtures) python "$ut/bspwalk/run_bspwalk_tests.py" --walker-args=-vischeck ;;
    bspwalk_cache)    python -I "$ut/bspwalk/run_bspwalk_cache_tests.py" ;;
    lmoptimizer)      python -m pytest "$ut/lmoptimizer" -q ;;
    proplm)           python -m pytest "$ut/proplm" -q ;;
    vvis_sightline)   python -I "$ut/vvis_sightline/run_sightline_tests.py" ;;
    vvis_mergeflow)   python -I "$ut/vvis_mergeflow/run_mergeflow_tests.py" ;;
    *) echo "unknown suite $1"; return 2 ;;
  esac
}

failed=0
summary=""
for s in $only; do
  start=$(date +%s)
  run_suite "$s" > "$logdir/$s.log" 2>&1
  rc=$?
  secs=$(( $(date +%s) - start ))
  if [ "$rc" -eq 0 ]; then verdict=PASS; else verdict=FAIL; failed=1; fi
  line=$(printf '%-17s %s  rc=%d  %4ds  %s' "$s" "$verdict" "$rc" "$secs" "$(grep -v '^[[:space:]]*$' "$logdir/$s.log" | tail -1 | cut -c1-110)")
  echo "$line"
  summary="$summary$line"$'\n'
done
printf '%s' "$summary" > "$logdir/summary.txt"
echo "logs: $logdir"
exit $failed
