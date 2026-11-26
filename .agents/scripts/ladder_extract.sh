#!/usr/bin/env bash
# Run a change made in a review extract against a real build of this repository.
#
# Why: the container implementer and the review lanes work from a snapshot, and
# a snapshot of the whole repository is too large to unpack on the review host,
# so `ladder-implement` is given a small extract (review_extract.sh) as its
# `repo`. An extract cannot be built, so its host check has nowhere to run.
# This script mirrors the change made in an extract worktree into a real
# worktree of the main repository, then builds and checks there.
#
# Usage:
#   bash .agents/scripts/ladder_extract.sh make <out_dir> <regex>...
#   bash .agents/scripts/ladder_extract.sh build [worktree_build_setup.sh args]
#   bash .agents/scripts/ladder_extract.sh run '<command line>'
#   bash .agents/scripts/ladder_extract.sh where
#
#   make   (from inside this repository) builds an extract of HEAD through
#          `review_extract.sh <out_dir> HEAD HEAD <regex>...` and records in its
#          git config where it came from (extract.source = this repository's
#          top level, extract.base = HEAD's sha). Every git worktree of the
#          extract shares that config, so the worktrees `ladder-implement` makes
#          with `git worktree add --detach` can find the source. The extract
#          mirrors HEAD, not uncommitted edits in the working tree.
#   build  (from an extract or any worktree of one) makes the sibling worktree
#          <extract worktree>-real of the source repository at extract.base, if
#          it is missing; takes the change in the extract worktree (`git diff
#          xbase`, untracked files that are not ignored included, binary-safe);
#          reverse-applies the patch an earlier build applied and applies this
#          one; then runs worktree_build_setup.sh in the real worktree with the
#          arguments given (none: only copy the untracked inputs and generate
#          the projects; `--project vbsp`: also build that project).
#          The last applied patch is kept as <real>.applied.diff. Only the files
#          that patch names are ever touched: the real worktree holds untracked
#          libraries and build output that must survive, so there is no clean
#          and no checkout of the whole tree.
#   run    runs the command line with bash from the top of the real worktree and
#          exits with its code; it is the acceptance check.
#   where  prints the path of the real worktree. Remove the pair afterwards with
#          `git -C <source> worktree remove --force <real>`.
#
# Exit codes: 0 success; 2 usage, missing extract config or an unusable real
# worktree; 3 the change does not apply (or an earlier patch cannot be taken
# back because the real worktree was edited by hand); otherwise the build's or
# the command's own code.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
# The lines this script prints for a caller to paste are read by Node-based
# tools too, which do not know Git Bash's /tmp or /c/ mounts: print C:/... form.
winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s\n' "$1"; fi; }
self=$(winpath "$here/$(basename "$0")")

usage() { # <exit code>
  awk 'NR > 1 { if (/^#/) { sub(/^# ?/, ""); print } else exit }' "$0" >&2
  exit "${1:-2}"
}
die() { # <message> [exit code]
  echo "ladder_extract: $1" >&2
  exit "${2:-1}"
}

# Sets top, src_repo, base and real for the extract the shell is in.
locate() {
  top=$(git rev-parse --show-toplevel 2>/dev/null) || die "not inside a git repository" 2
  src_repo=$(git -C "$top" config --get extract.source 2>/dev/null || true)
  base=$(git -C "$top" config --get extract.base 2>/dev/null || true)
  if [ -z "$src_repo" ] || [ -z "$base" ]; then
    die "$top is not an extract made by 'ladder_extract.sh make': extract.source and extract.base are not set in its git config" 2
  fi
  real="$top-real"
}

cmd_make() {
  [ "$#" -ge 2 ] || usage 2
  local out=$1 src head
  shift
  src=$(git rev-parse --show-toplevel 2>/dev/null) || die "make runs from inside the repository to mirror" 2
  head=$(git rev-parse HEAD)
  bash "$here/review_extract.sh" "$out" HEAD HEAD "$@"
  out=$(git -C "$out" rev-parse --show-toplevel)
  git -C "$out" config extract.source "$src"
  git -C "$out" config extract.base "$head"
  echo
  echo "extract: $out"
  echo "  source $src at $head"
  echo "ladder-implement: repo = $out, and"
  echo "  build = bash \"$self\" build --project <name>"
  echo "  check = bash \"$self\" run '<check>'"
}

cmd_build() {
  locate
  local applied="$real.applied.diff" pending="$real.pending.diff" idx prefix
  git -C "$src_repo" rev-parse --git-dir >/dev/null 2>&1 \
    || die "the source repository $src_repo (extract.source) is gone" 2
  git -C "$src_repo" cat-file -e "$base^{commit}" 2>/dev/null \
    || die "commit $base (extract.base) is not in $src_repo" 2

  if [ ! -e "$real" ]; then
    # A patch kept for a worktree that no longer exists would be taken back
    # from a tree that never had it.
    rm -f "$applied"
    # A real worktree deleted by hand is still registered; git refuses to add
    # it again until the stale entry is pruned.
    git -C "$src_repo" worktree prune
    git -C "$src_repo" worktree add --quiet --detach "$real" "$base" \
      || die "could not create the real worktree $real" 1
    echo "[ladder] created $real at ${base:0:10}"
  fi
  prefix=$(git -C "$real" rev-parse --show-prefix 2>/dev/null) && [ -z "$prefix" ] \
    || die "$real exists but is not the top of a git worktree" 2
  [ "$(git -C "$real" rev-parse HEAD)" = "$base" ] \
    || die "$real is at $(git -C "$real" rev-parse --short HEAD), not at extract.base ${base:0:10}; remove it and build again" 2

  # The change against xbase, whatever commit this worktree sits at (it may
  # already hold an earlier patch). A scratch index lets `add -A` pick up the
  # untracked files without touching this worktree's own index.
  idx=$(mktemp -u)
  trap 'rm -f "$idx" "$pending"' EXIT
  GIT_INDEX_FILE=$idx git -C "$top" read-tree xbase
  GIT_INDEX_FILE=$idx git -C "$top" add -A
  GIT_INDEX_FILE=$idx git -C "$top" diff --cached --binary --no-renames xbase > "$pending"

  # The extract keeps the line endings review_extract.sh checked out (CRLF for
  # text files on Windows) while the real worktree's follow its own autocrlf,
  # so a hunk whose context differs from the file only in a trailing CR must
  # still apply. --whitespace=nowarn: added lines are taken as written.
  local ap=(--ignore-whitespace --whitespace=nowarn)
  if [ -s "$applied" ]; then
    if git -C "$real" apply -R --check "${ap[@]}" "$applied" 2>/dev/null; then
      git -C "$real" apply -R "${ap[@]}" "$applied"
    elif git -C "$real" apply --check "${ap[@]}" "$applied" 2>/dev/null; then
      # A build that was killed after recording the patch and before applying
      # it (see below): the tree is already at the base for these files.
      echo "[ladder] the recorded patch was never applied to $real (an interrupted build); dropping the record"
    else
      die "the patch of the earlier build cannot be taken back from $real: the files it names were edited there. Undo those edits (or remove the worktree and the file $applied) and build again" 3
    fi
    rm -f "$applied"
  fi
  if [ -s "$pending" ]; then
    # Record first, apply second: builds run in the background and get killed
    # by timeouts, and a record without its patch is recognised above, while a
    # patch without its record could never be taken back.
    cp -f "$pending" "$applied"
    git -C "$real" apply "${ap[@]}" "$pending" \
      || { rm -f "$applied"; die "the change in $top does not apply to $real at ${base:0:10}" 3; }
    rm -f "$pending"
    echo "[ladder] applied to $real:"
    git -C "$real" apply --stat "$applied" | sed 's/^/  /'
  else
    echo "[ladder] no change against xbase in $top; $real is at the base"
  fi

  local rc=0
  (cd "$real" && bash .agents/scripts/worktree_build_setup.sh "$@") || rc=$?
  exit "$rc"
}

cmd_run() {
  [ "$#" -ge 1 ] || usage 2
  locate
  [ -d "$real" ] || die "no real worktree at $real: run '$self build' first" 2
  cd "$real"
  exec bash -c "$*"
}

cmd_where() {
  locate
  echo "$real"
}

[ "$#" -ge 1 ] || usage 2
cmd=$1
shift
case "$cmd" in
  make) cmd_make "$@" ;;
  build) cmd_build "$@" ;;
  run) cmd_run "$@" ;;
  where) cmd_where ;;
  -h|--help) usage 0 ;;
  *) usage 2 ;;
esac
