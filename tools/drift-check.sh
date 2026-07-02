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

# ── Flash-usage check ──────────────────────────────────────────────────────
# Surface how full the OTA app slot is so slow binary growth toward the
# partition cap is visible at a glance. Best-effort: needs a built .bin plus a
# partitions.csv, and is silently skipped if either is missing. The slot size
# is read from partitions.csv (app0) so it tracks any future repartition.
# Always warns (even under --quiet) once usage crosses FLASH_WARN_PCT.
FLASH_WARN_PCT=90
bin=$(ls -t build/*/*.ino.bin 2>/dev/null | head -1)
slot_hex=$(awk -F',' '/^[[:space:]]*app0[[:space:]]*,/{gsub(/[[:space:]]/,"",$5); print $5}' partitions.csv 2>/dev/null)
if [ -n "$bin" ] && [ -n "$slot_hex" ]; then
    bin_size=$(wc -c < "$bin")
    slot=$((slot_hex))
    if [ "$slot" -gt 0 ]; then
        pct=$((bin_size * 100 / slot))
        free_kb=$(((slot - bin_size) / 1024))
        flash_msg="flash: ${pct}% of OTA slot (${free_kb} KB free)"
        if [ "$pct" -ge "$FLASH_WARN_PCT" ]; then
            echo "${YEL}${flash_msg} — nearing cap${RST}"
        elif [ "$QUIET" = 0 ]; then
            echo "${DIM}${flash_msg}${RST}"
        fi
    fi
fi

if [ -z "$released" ]; then
    [ "$QUIET" = 1 ] || echo "${YEL}gh-pages has no release yet${RST} — current: v$current"
    [ "$CHECK" = 1 ] && exit 1
    exit 0
fi

rel_major=$(echo "$released" | cut -d. -f1)
rel_minor=$(echo "$released" | cut -d. -f2)
released_patch=$(echo "$released" | cut -d. -f3)

# Guard against a malformed manifest.json (non-numeric fields). Don't error —
# the script is a status check, not a validator.
if ! [[ "$rel_major" =~ ^[0-9]+$ ]] || ! [[ "$rel_minor" =~ ^[0-9]+$ ]] \
   || ! [[ "$released_patch" =~ ^[0-9]+$ ]] || ! [[ "$patch" =~ ^[0-9]+$ ]]; then
    [ "$QUIET" = 1 ] || echo "${YEL}gh-pages manifest version is malformed${RST}: $released"
    exit 0
fi

# Compare the FULL version tuple — comparing patch alone inverts the verdict
# right after a minor bump (12.3.32 vs 12.4.5 must read "behind", not "ahead").
rel_score=$(( (rel_major * 1000 + rel_minor) * 100000 + released_patch ))
cur_score=$(( (v_major * 1000 + v_minor) * 100000 + patch ))

if [ "$rel_score" -eq "$cur_score" ]; then
    [ "$QUIET" = 1 ] || echo "${GRN}gh-pages in sync${RST} — v$released"
    exit 0
elif [ "$rel_score" -lt "$cur_score" ]; then
    if [ "$rel_major" -ne "$v_major" ] || [ "$rel_minor" -ne "$v_minor" ]; then
        # Across a minor bump the patch counters aren't comparable — the commit
        # gap can't be derived from versions alone. It's unambiguously drift.
        echo "${YEL}gh-pages is behind main (new minor release pending)${RST}"
        echo "${DIM}  released: v$released${RST}"
        echo "${DIM}  current : v$current${RST}"
        echo "${DIM}  ship:     bash tools/release.sh --yes${RST}"
        [ "$CHECK" = 1 ] && exit 1
        exit 0
    fi
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
