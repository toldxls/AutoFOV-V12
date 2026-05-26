#!/bin/bash
# Compare main HEAD vs the last released firmware on gh-pages.
# Reads the released version from gh-pages's manifest.json and the current
# patch number from `git rev-list --count HEAD` (same recipe tools/embed_html.py
# uses), so the gap is meaningful regardless of which branch is checked out.
#
# Always exits 0 by default — safe to source from a shell precmd or cd hook.
# Pass --check to exit non-zero on drift (e.g. for a pre-push hook).
#
# Usage:
#   bash tools/drift-check.sh           # always print, exit 0
#   bash tools/drift-check.sh --quiet   # print only when drifted, exit 0
#   bash tools/drift-check.sh --check   # exit 1 on drift (for hooks)

set -e
cd "$(dirname "$0")/.."

QUIET=0
CHECK=0
for arg in "$@"; do
    case "$arg" in
        --quiet|-q) QUIET=1 ;;
        --check|-c) CHECK=1 ;;
        --help|-h)
            sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
    esac
done

# ANSI colours only when stdout is a tty (don't poison logs / hook stdouts).
if [ -t 1 ]; then
    YEL=$(printf '\033[33m'); GRN=$(printf '\033[32m')
    DIM=$(printf '\033[2m');  RST=$(printf '\033[0m')
else
    YEL=''; GRN=''; DIM=''; RST=''
fi

# Best-effort refresh — offline still produces useful output from cached tracking ref.
git fetch origin gh-pages --quiet 2>/dev/null || true

# Read the version field from gh-pages's manifest.json without checking out the branch.
released=""
if git show-ref --verify --quiet refs/remotes/origin/gh-pages; then
    released=$(git show origin/gh-pages:manifest.json 2>/dev/null \
               | sed -n 's/.*"version"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')
fi

commit_count=$(git rev-list --count HEAD)

# Mirror tools/embed_html.py's version formula. Reading the constants out of
# that file keeps the two in lockstep — bumping VERSION_MINOR or rebasing
# VERSION_PATCH_BASE there updates this check automatically.
embed_py="tools/embed_html.py"
v_major=$(sed -n 's/^VERSION_MAJOR[[:space:]]*=[[:space:]]*\([0-9]\{1,\}\).*/\1/p' "$embed_py")
v_minor=$(sed -n 's/^VERSION_MINOR[[:space:]]*=[[:space:]]*\([0-9]\{1,\}\).*/\1/p' "$embed_py")
v_base=$(sed -n 's/^VERSION_PATCH_BASE[[:space:]]*=[[:space:]]*\([0-9]\{1,\}\).*/\1/p' "$embed_py")
: "${v_major:=12}" "${v_minor:=0}" "${v_base:=0}"
patch=$((commit_count - v_base))
[ "$patch" -lt 0 ] && patch=0
current="$v_major.$v_minor.$patch"

if [ -z "$released" ]; then
    [ "$QUIET" = 1 ] || echo "${YEL}gh-pages has no release yet${RST} — current: v$current"
    [ "$CHECK" = 1 ] && exit 1
    exit 0
fi

released_patch=${released##*.}

# Guard against a malformed manifest.json (non-numeric patch). Don't error —
# the script is a status check, not a validator.
if ! [[ "$released_patch" =~ ^[0-9]+$ ]] || ! [[ "$patch" =~ ^[0-9]+$ ]]; then
    [ "$QUIET" = 1 ] || echo "${YEL}gh-pages manifest version is malformed${RST}: $released"
    exit 0
fi

if [ "$released_patch" -eq "$patch" ]; then
    [ "$QUIET" = 1 ] || echo "${GRN}gh-pages in sync${RST} — v$released"
    exit 0
elif [ "$released_patch" -lt "$patch" ]; then
    gap=$((patch - released_patch))

    # Suppress the false positive every release naturally produces: release.sh
    # regenerates web_ui.h (because BUILD_COMMIT changed) and that artifact
    # commit lands on main *after* the version it documents went to gh-pages.
    # If every commit since the release is one of those regen commits, the
    # firmware behaviour is identical to what's already shipped — not drift.
    real_commits=$(git log --pretty=%s "HEAD~${gap}..HEAD" 2>/dev/null \
                   | grep -vc '^build: regenerate web_ui\.h' || true)
    if [ "$real_commits" -eq 0 ]; then
        [ "$QUIET" = 1 ] || echo "${GRN}gh-pages in sync${RST} — v$released ${DIM}(+ release-artifact regen)${RST}"
        exit 0
    fi

    plural=""; [ "$real_commits" -ne 1 ] && plural="s"
    echo "${YEL}gh-pages is $real_commits real commit$plural behind main${RST}"
    echo "${DIM}  released: v$released${RST}"
    echo "${DIM}  current : v$current${RST}"
    echo "${DIM}  ship:     bash tools/release.sh --yes${RST}"
    [ "$CHECK" = 1 ] && exit 1
    exit 0
else
    # gh-pages ahead of main — happens if a release was pushed from another
    # machine you haven't pulled from yet. Surface it but don't treat as drift.
    [ "$QUIET" = 1 ] || echo "${YEL}gh-pages is AHEAD of main${RST} — released v$released, main v$current"
    exit 0
fi
