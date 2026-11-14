#!/usr/bin/env bash
# Make a fresh git worktree of this repository buildable, then optionally build.
#
# Why: the prebuilt libs under src/lib, the runtime DLLs in game/bin/x64 and
# some test maps are untracked, so a new worktree cannot link or run tools
# until they are copied from the main checkout; and createallprojects.bat run
# from a worktree can write the solution without its .sln extension.
#
# Usage (from anywhere inside the worktree):
#   bash .agents/scripts/worktree_build_setup.sh [--build] [--project <name>]...
#
#   (no option)        copy the missing untracked inputs and generate the projects
#   --build            then build the whole solution (Release, win64)
#   --project <name>   then build only src/utils/<name>/*.vcxproj (repeatable),
#                      e.g. --project vbsp --project vvis. Its library
#                      dependencies must already be built or copied.
#
# Copies never overwrite (cp -n). Re-run it after a .vpc changes: it always
# regenerates the projects. Build output: src/build_tools.log (whole solution)
# or the console (single projects). Exit code: 0 on success.
set -euo pipefail

build_all=0
projects=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --build) build_all=1; shift ;;
    --project) projects+=("$2"); shift 2 ;;
    *) sed -n '2,20p' "$0" >&2; exit 2 ;;
  esac
done

root=$(git rev-parse --show-toplevel)
root=$(cd "$root" && pwd)
main=$(cd "$(git rev-parse --git-common-dir)/.." && pwd)
lzma=src/lib/common/x64/2015/release/lzma.lib
# A build rewrites this tracked lib; put it back afterwards unless it already
# carried a local change.
lzma_clean=0
git -C "$root" diff --quiet -- "$lzma" && lzma_clean=1

if [ "$root" != "$main" ]; then
  # Untracked build and run inputs, taken from the main checkout.
  (cd "$main" && git ls-files -oi --exclude-standard src/lib game/bin/x64/unit_test_maps \
      | grep -v '\.pdb$' || true) | while IFS= read -r f; do
    mkdir -p "$root/$(dirname "$f")"
    cp -n "$main/$f" "$root/$f"
  done
  for f in tier0.dll vstdlib.dll filesystem_stdio.dll; do
    [ -f "$main/game/bin/x64/$f" ] && cp -n "$main/game/bin/x64/$f" "$root/game/bin/x64/$f"
  done
  echo "[setup] untracked inputs copied from $main"
fi

cd "$root/src"
cmd //c ".\\createallprojects.bat" > createallprojects.log 2>&1 || { tail -20 createallprojects.log; exit 1; }
# From a worktree vpc can write the solution without its extension; that file
# is not git-ignored, so never leave it behind.
[ -f mapping_tools ] && mv -f mapping_tools mapping_tools.sln
[ -f mapping_tools.sln ] || { echo "[setup] no mapping_tools.sln was generated" >&2; exit 1; }
echo "[setup] projects generated"

msbuild=$("/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" -latest \
  -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | head -1)

if [ "$build_all" -eq 1 ]; then
  "$msbuild" "$(cygpath -w "$root/src/mapping_tools.sln")" -p:Configuration=Release -p:Platform=win64 \
    -m -t:Build -nodeReuse:false -fl "-flp:LogFile=build_tools.log;Verbosity=normal" \
    "-clp:Summary;Verbosity=minimal" | tail -15
fi
for p in "${projects[@]}"; do
  for proj in "$root/src/utils/$p"/*.vcxproj; do
    "$msbuild" "$(cygpath -w "$proj")" -p:Configuration=Release -p:Platform=x64 \
      -nodeReuse:false "-clp:Summary;Verbosity=minimal" | tail -15
  done
done
if [ "$lzma_clean" -eq 1 ]; then git -C "$root" checkout -- "$lzma" 2>/dev/null || true; fi
