#!/usr/bin/env bash
# Build a small two-commit git repo holding only the files a review needs.
#
# Why: review_fast / review_final snapshot the whole repository (about 711 MB
# here) and the unpack on the review host times out, so every lane ends
# launch_failed. A review of an extract launches in seconds.
#
# Usage (from anywhere inside this repository):
#   bash .agents/scripts/review_extract.sh <out_dir> <base_rev> <head_rev> <regex>...
#
# Each <regex> is an extended regex matched against repo-relative paths, e.g.
#   '^src/utils/vvis/'  '^game/bin/x64/[^/]+\.py$'
# Files matching any regex are taken, at base_rev for the first commit and at
# head_rev for the second. Then review with:
#   repo = <out_dir>, diff_range = xbase..xhead   (both are tags in the extract)
# and tell the reviewers the repo is an extract that does not build.
set -euo pipefail

if [ "$#" -lt 4 ]; then
  sed -n '2,17p' "$0" >&2
  exit 2
fi

out=$1; base=$2; head=$3; shift 3
src=$(git rev-parse --show-toplevel)
pattern=$(printf '%s\n' "$@" | paste -sd'|' -)

if [ -e "$out" ] && [ -n "$(ls -A "$out" 2>/dev/null)" ]; then
  echo "refusing to write into non-empty $out" >&2
  exit 1
fi
mkdir -p "$out"
out=$(cd "$out" && pwd)
git -C "$out" init -q
git -C "$out" config core.autocrlf false
git -C "$out" config user.name "review extract"
git -C "$out" config user.email "extract@localhost"

fill() { # <rev> <tag>
  local rev=$1 tag=$2 idx n
  idx=$(mktemp)
  rm -f "$idx"
  git -C "$out" rm -rq --ignore-unmatch . >/dev/null 2>&1 || true
  n=$(git -C "$src" ls-tree -r "$rev" | PAT="$pattern" awk -F'\t' '$2 ~ ENVIRON["PAT"]' | tee "$idx.list" | wc -l)
  if [ "$n" -eq 0 ]; then
    echo "no file at $rev matches: $pattern" >&2
    exit 1
  fi
  GIT_INDEX_FILE=$idx git -C "$src" update-index --add --index-info < "$idx.list"
  GIT_INDEX_FILE=$idx git -C "$src" -c core.autocrlf=false checkout-index -a -f --prefix="$out/"
  rm -f "$idx" "$idx.list"
  git -C "$out" add -A
  git -C "$out" commit -q --allow-empty -m "$tag: $(git -C "$src" rev-parse --short "$rev") of $(basename "$src") (extract)"
  git -C "$out" tag -f "$tag" >/dev/null
  echo "$tag: $n files from $(git -C "$src" rev-parse --short "$rev")"
}

fill "$base" xbase
fill "$head" xhead
git -C "$out" diff --shortstat xbase..xhead
echo "extract at $out ($(du -sh "$out/.git" | cut -f1) of git data); review with diff_range xbase..xhead"
