#!/usr/bin/env bash
# Build and run this repository's tools on Server1 instead of the dev box.
#
# Server1 (ssh host "server1"): Windows Server 2019, 12 threads, VS 2026,
# CUDA 13.4, a GTX TITAN X, Python 3.14, HL2DM and Source SDK Base 2013 under
# C:\Steam\steamapps\common. Its login shell is PowerShell; this script feeds
# Git for Windows' bash over stdin. The checkout lives in C:\Dev\semt\repo.
#
# Usage (from anywhere inside this repository on the dev box):
#   bash .agents/scripts/server1.sh setup        one time: clone, copy the untracked libs,
#                                                DLLs and test maps, pip install pytest
#   bash .agents/scripts/server1.sh sync [rev]   put <rev> (default HEAD, pushed or not) on
#                                                Server1 as a detached checkout
#   bash .agents/scripts/server1.sh run '<bash command line>'
#                                                run it in the checkout's root, e.g.
#       run 'bash .agents/scripts/worktree_build_setup.sh --build'
#       run 'bash .agents/scripts/run_suites.sh --only proplm'
#   bash .agents/scripts/server1.sh fetch <remote path under the checkout> <local dir>
#
# "run" sets SEMT_STEAM_COMMON=C:/Steam/steamapps/common, which the Python
# harnesses use for their default game dirs (HL2DM and Source SDK Base 2013
# Multiplayer are installed there). Black Mesa and the other content sets of
# prop_corpus_check.py are not on Server1.
# Exit code: the remote command's.
set -euo pipefail

host=server1
rdir=/c/Dev/semt/repo
rbash="& 'C:\\Program Files\\Git\\bin\\bash.exe' -s"
url=https://github.com/legoj15/source-extreme-mapping-tools.git
root=$(git rev-parse --show-toplevel)
main=$(cd "$(git rev-parse --git-common-dir)/.." && pwd)

remote() { ssh -o BatchMode=yes "$host" "$rbash" 2> >(grep -v -e post-quantum -e 'store now' -e 'may need to be upgraded' >&2); }

cmd=${1:-}; shift || true
case "$cmd" in
  setup)
    echo "mkdir -p /c/Dev/semt && cd /c/Dev/semt && { [ -d repo/.git ] || git clone -q --depth 1 $url repo; } && python -m pip install -q --user pytest 2>&1 | tail -1" | remote
    # Untracked build and run inputs from the main checkout (never overwritten).
    (cd "$main" && { git ls-files -oi --exclude-standard src/lib game/bin/x64/unit_test_maps | grep -v '\.pdb$'
       printf '%s\n' game/bin/x64/tier0.dll game/bin/x64/vstdlib.dll game/bin/x64/filesystem_stdio.dll; } \
       | tar -cf - -T -) | ssh -o BatchMode=yes "$host" "& 'C:\\Program Files\\Git\\usr\\bin\\tar.exe' -xkf - -C C:/Dev/semt/repo" 2>/dev/null || true
    echo "ls $rdir/src/lib/public/x64 | wc -l; ls $rdir/game/bin/x64/*.dll" | remote
    ;;
  sync)
    rev=$(git rev-parse "${1:-HEAD}")
    b=$(mktemp); trap 'rm -f "$b"' EXIT
    if git merge-base --is-ancestor "$rev" origin/master 2>/dev/null; then
      echo "cd $rdir && git fetch -q --depth 50 origin master && git checkout -q --detach $rev && git log --oneline -1" | remote
    else
      # Not pushed: send the commits past origin/master as a bundle.
      # A bundle needs a named ref, so park the commit under one.
      git update-ref refs/semt/sync "$rev"
      git bundle create -q "$b" origin/master..refs/semt/sync
      scp -q -o BatchMode=yes "$b" "$host:C:/Dev/semt/sync.bundle" 2>/dev/null
      echo "cd $rdir && git fetch -q --depth 50 origin master && git fetch -q ../sync.bundle refs/semt/sync && git checkout -q --detach $rev && git log --oneline -1" | remote
    fi
    ;;
  run)
    echo "cd $rdir && export SEMT_STEAM_COMMON='C:/Steam/steamapps/common' && $*" | remote
    ;;
  fetch)
    mkdir -p "$2"
    scp -q -r -o BatchMode=yes "$host:C:/Dev/semt/repo/$1" "$2" 2>/dev/null
    ;;
  *) sed -n '2,24p' "$0" >&2; exit 2 ;;
esac
