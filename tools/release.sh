#!/bin/bash
# Build a release and publish it to GitHub Pages.
#
# Pages hosts what the dashboard's "Update from GitHub" button and the USB
# recovery flasher need:
#   firmware.bin          — the app image (also flashed at 0x10000 on recovery)
#   bootloader.bin        — recovery: flashed at 0x0
#   partitions.bin        — recovery: flashed at 0x8000
#   boot_app0.bin         — recovery: flashed at 0xe000 (otadata reset)
#   manifest.json         — {version, sha256, bin} for the OTA update check
#   recovery-manifest.json— ESP Web Tools install manifest
#   index.html            — the recovery flasher page (site root)
#
# These are committed to the `gh-pages` branch via a git worktree, so the
# main working tree is never touched.  First run bootstraps the branch.
#
# Usage:
#   bash tools/release.sh           # interactive — prompts before pushing
#   bash tools/release.sh --yes     # non-interactive — pushes without prompt
#                                   # (used after the OTA flash has been tested)

set -e
cd "$(dirname "$0")/.."

AUTO_YES=0
[ "${1:-}" = "--yes" ] || [ "${1:-}" = "-y" ] && AUTO_YES=1

BUILD_DIR="build/esp32.esp32.adafruit_feather_esp32s3"
PAGES_URL="https://toldxls.github.io/AutoFOV-V12"
# gh-pages is a deploy target, not an archive — every release adds a fresh
# ~1.5 MB firmware.bin that git keeps forever (binaries don't delta-compress).
# After each release we prune gh-pages history to the last N releases so the
# branch stays flat: recent firmware.bin kept as a backup, older blobs dropped
# (reclaimed by GitHub's server-side gc). The real history lives on main.
KEEP_RELEASES=5

# Refuse a dirty tree: the version is stamped from HEAD's commit count, so a
# dirty build publishes a binary that corresponds to no commit — and two
# different binaries can ship under the same version number. data/web_ui.h is
# exempt (build.sh regenerates it; its post-release commit is the documented
# "build: regenerate web_ui.h" follow-up).
DIRTY=$(git status --porcelain | grep -v ' data/web_ui\.h$' || true)
if [ -n "$DIRTY" ]; then
    echo "ERROR: working tree not clean — commit first (version is stamped from HEAD):"
    echo "$DIRTY"
    exit 1
fi

# ── Step 1: build (embeds HTML + compiles) ───────────────────────────────────
echo "=== Step 1: build ==="
bash tools/build.sh

# ── Step 2: gather artifacts ─────────────────────────────────────────────────
echo ""
echo "=== Step 2: collect artifacts ==="

VERSION=$(sed -n 's/.*BUILD_VERSION "\([^"]*\)".*/\1/p' data/web_ui.h)
[ -n "$VERSION" ] || { echo "ERROR: could not read BUILD_VERSION from data/web_ui.h"; exit 1; }

APP_BIN="$BUILD_DIR/AutoFOV_V12.ino.bin"
BOOT_BIN="$BUILD_DIR/AutoFOV_V12.ino.bootloader.bin"
PART_BIN="$BUILD_DIR/AutoFOV_V12.ino.partitions.bin"
# boot_app0.bin ships with the ESP32 Arduino core — pick the newest installed.
BOOTAPP0=$(ls -1 "$HOME"/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin 2>/dev/null | sort -V | tail -1)

for f in "$APP_BIN" "$BOOT_BIN" "$PART_BIN" "$BOOTAPP0"; do
    [ -f "$f" ] || { echo "ERROR: missing build artifact: $f"; exit 1; }
done

SHA=$(shasum -a 256 "$APP_BIN" | awk '{print $1}')
APP_SIZE=$(wc -c < "$APP_BIN" | tr -d ' ')

echo "version : v$VERSION"
echo "app bin : $APP_SIZE bytes"
echo "sha256  : $SHA"

# ── Step 3: stage the gh-pages worktree ──────────────────────────────────────
echo ""
echo "=== Step 3: stage gh-pages ==="

WT=$(mktemp -d)
cleanup() { git worktree remove --force "$WT" 2>/dev/null || true; rm -rf "$WT"; }
trap cleanup EXIT

# Heal any stale worktree from an old aborted run — a registered worktree
# holding gh-pages makes the `worktree add -B` below fail forever. prune only
# covers deleted directories; the old abort path KEPT its directory, so also
# explicitly remove whichever worktree has gh-pages checked out.
OLDWT=$(git worktree list --porcelain | awk '/^worktree /{wt=substr($0,10)} /^branch refs\/heads\/gh-pages$/{print wt}')
if [ -n "$OLDWT" ]; then
    echo "Removing stale gh-pages worktree from an aborted run: $OLDWT"
    git worktree remove --force "$OLDWT" 2>/dev/null || true
fi
git worktree prune 2>/dev/null || true

git fetch origin gh-pages:refs/remotes/origin/gh-pages 2>/dev/null || true
if git show-ref --verify --quiet refs/remotes/origin/gh-pages; then
    # Remote branch exists — reset the local gh-pages to match it (the main
    # tree never commits there, so a hard reset is always correct).
    git worktree add -B gh-pages "$WT" refs/remotes/origin/gh-pages
elif git show-ref --verify --quiet refs/heads/gh-pages; then
    git worktree add "$WT" gh-pages
else
    echo "gh-pages branch not found — bootstrapping a new one."
    git worktree add --detach "$WT"
    ( cd "$WT" && git checkout --orphan gh-pages )
fi
# Always clear the worktree before staging the release, so files dropped from
# the published set (e.g. a renamed page) actually disappear from gh-pages
# rather than lingering forever.
( cd "$WT" && git rm -rf --quiet . 2>/dev/null || true )

cp "$APP_BIN"  "$WT/firmware.bin"
cp "$BOOT_BIN" "$WT/bootloader.bin"
cp "$PART_BIN" "$WT/partitions.bin"
cp "$BOOTAPP0" "$WT/boot_app0.bin"
cp web/recovery.html "$WT/index.html"   # served at the Pages site root
: > "$WT/.nojekyll"   # serve files verbatim — no Jekyll processing

# V12.3: publish the owner-curated calibration library (dashboard CAL I/O →
# LOAD FROM GITHUB reads calibrations/index.json). Copy ONLY the top-level
# library files (*.json, README) — NOT photo subfolders that may live under
# calibrations/ locally as test backups (gitignored on main; would otherwise
# bloat gh-pages by hundreds of MB).
[ -d calibrations ] && mkdir -p "$WT/calibrations" && \
  find calibrations -maxdepth 1 -type f \( -name '*.json' -o -name '*.md' \) \
    -exec cp {} "$WT/calibrations/" \;

cat > "$WT/manifest.json" <<EOF
{ "version": "$VERSION", "sha256": "$SHA", "bin": "firmware.bin" }
EOF

# ESP Web Tools manifest — multi-part flash that leaves the NVS region (0x9000)
# untouched, so a recovery flash preserves calibration and WiFi settings.
cat > "$WT/recovery-manifest.json" <<EOF
{
  "name": "AutoFOV V12",
  "version": "$VERSION",
  "builds": [
    {
      "chipFamily": "ESP32-S3",
      "parts": [
        { "path": "bootloader.bin", "offset": 0 },
        { "path": "partitions.bin", "offset": 32768 },
        { "path": "boot_app0.bin",  "offset": 57344 },
        { "path": "firmware.bin",   "offset": 65536 }
      ]
    }
  ]
}
EOF

# ── Step 4: commit + push ────────────────────────────────────────────────────
echo ""
echo "=== Step 4: publish ==="
( cd "$WT" && git add -A && git status --short )

if [ "$AUTO_YES" = 1 ]; then
    ans=y
    echo ""
    echo "--yes given — pushing v$VERSION without prompting."
else
    printf "\nPush v%s to the gh-pages branch on origin? [y/N] " "$VERSION"
    read -r ans
fi
if [ "$ans" = "y" ] || [ "$ans" = "Y" ]; then
    (
      cd "$WT"
      git commit -qm "release v$VERSION"
      REL=$(git rev-parse HEAD)
      # Prune gh-pages to the last $KEEP_RELEASES releases (see note above). Rebuild
      # a fresh linear history of just those commits' TREES via plumbing — no
      # rebase, no conflicts. Best-effort with a hard safety gate: the pruned tip
      # MUST serve the exact release tree, else we keep full history. bash-3.2 safe.
      set +e
      n=$(git rev-list --count HEAD)
      if [ "$n" -gt "$KEEP_RELEASES" ]; then
        prev=""
        for c in $(git rev-list -n "$KEEP_RELEASES" HEAD | awk '{a[NR]=$0} END{for(i=NR;i>=1;i--)print a[i]}'); do
          t=$(git rev-parse "$c^{tree}"); m=$(git log -1 --format=%s "$c")
          if [ -z "$prev" ]; then prev=$(git commit-tree "$t" -m "$m")
          else                    prev=$(git commit-tree "$t" -p "$prev" -m "$m"); fi
        done
        if [ -n "$prev" ] && [ "$(git rev-parse "$prev^{tree}" 2>/dev/null)" = "$(git rev-parse "$REL^{tree}")" ]; then
          git checkout -qB gh-pages "$prev"
          echo "gh-pages history pruned to the last $KEEP_RELEASES releases (was $n commits)"
        else
          echo "(history prune skipped — content safety check failed; full history kept)"
        fi
      fi
      set -e
      # Force needed after a prune (history rewrite); a no-prune push is a
      # fast-forward so -f is harmless. Content is guaranteed correct by the gate.
      git push -f origin gh-pages
    )
    # Deploy via the Actions workflow (Pages Source must be "GitHub Actions").
    # Replaces the legacy branch auto-deploy, which kept failing on GitHub's
    # backend. gh workflow run is now the SOLE publish mechanism, so a failure
    # here means the push landed but NOTHING deployed — surface it loudly rather
    # than the pipe-masked false success we had before. Capture status directly
    # (no pipe), not the pipe's exit code, and warn hard if gh is missing.
    DEPLOYED=0
    if command -v gh >/dev/null 2>&1; then
        echo "Triggering Pages deploy workflow…"
        if gh workflow run pages.yml; then
            DEPLOYED=1
        else
            echo "  !! gh workflow run FAILED — gh-pages is pushed but NOT deployed."
            echo "  !! Fix gh auth / ensure pages.yml is on origin/main, then run:"
            echo "  !!   gh workflow run pages.yml"
        fi
    else
        echo "  !! gh CLI not found — gh-pages is pushed but NOT deployed."
        echo "  !! Install gh (or set Pages Source back to a branch), then run:"
        echo "  !!   gh workflow run pages.yml"
    fi
    echo ""
    if [ "$DEPLOYED" = 1 ]; then
        echo "=== Deploy triggered (v$VERSION) ==="
        echo "Watch: gh run watch \$(gh run list --workflow=pages.yml -L1 --json databaseId -q '.[0].databaseId')"
    else
        echo "=== Pushed to gh-pages, but DEPLOY NOT triggered (see above) ==="
    fi
    echo "Live at: $PAGES_URL/   (verify: curl -s $PAGES_URL/manifest.json)"
else
    echo "Aborted — nothing pushed."
    # The worktree is cleaned up by the EXIT trap. It must NOT be kept: a
    # registered worktree with gh-pages checked out blocks the next run's
    # `git worktree add -B gh-pages` — and Enter at the [y/N] prompt (the
    # default) used to land here and poison every future release.
fi
