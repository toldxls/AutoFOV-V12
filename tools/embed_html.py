#!/usr/bin/env python3
"""Generate web_ui.h from data/index.html.
Run once after any change to index.html (or to bump VERSION_MINOR),
then recompile and OTA-flash the firmware.
Usage: python3 tools/embed_html.py
"""
import gzip, os, re, shutil, subprocess, tempfile

# ── Version ───────────────────────────────────────────────────────────────────
VERSION_MAJOR = 12
VERSION_MINOR = 5   # bump manually for milestone releases
# Patch counter resets to 0 at each minor bump. Set VERSION_PATCH_BASE to the
# FIRMWARE commit count at the bump commit so patch = firmware_count - base.
# "Firmware commits" = commits that touch any of FIRMWARE_PATHS (below) — the
# inputs that change the shipped binary. Counting by PATH, not by commit
# subject, means: a docs/tooling commit never bumps the version (no phantom
# "update available" for identical firmware), and a firmware change can never
# sneak through un-bumped because its subject happened to start with `docs:`
# (which shipped a changed dashboard under an unchanged version on 8/21/26).
# release.sh's follow-up `build: regenerate web_ui.h` commits touch only
# data/web_ui.h, so they are excluded by construction. JS semver compare
# orders by minor first, so the OTA gate sees 12.2.x > 12.1.y regardless of patch.
# 12.2: the bug-sweep series — calibration sync, vib inStack latch, OTA reboot
# overlay, auth reconnect, TOF signal smoothing.
# 12.3: photo-assisted calibration — measure the pixel count from a micrometer
# JPEG (auto tick detection, deskew, in-focus region, contrast-profile review).
# 2026-07-05: base rebased 293 -> 166 when the formula switched to real-commit
# counting (211 real commits at released v12.4.45: 211 - 166 = 45).
# 2026-07-06: base rebased 166 -> 155 when ci:/chore:/docs: tooling & docs
# commits were also excluded. 212 real commits at v12.4.57: 212 - 155 = 57.
# 12.5: deep per-stack vibration history — /vibhist binary endpoint, the web
# cull report (per-frame blur, timeline strip, threshold flagging, saved-report
# archive), SNTP stack-start stamp, stackStart/sp telemetry.
# 2026-07-19: base rebased 155 -> 236 at the 12.5 bump (236 real commits at
# v12.5.0).
# 2026-08-21: base rebased 236 -> 228 when counting switched from subject
# exclusion to FIRMWARE_PATHS (307 firmware commits at v12.5.79: 307 - 228 = 79).
VERSION_PATCH_BASE = 228
# Paths whose commits change the shipped binary — the version counts ONLY
# these. ONE single-quoted line: drift-check.sh and release.sh sed it out of
# this file (same trick as VERSION_*), so keep the format.
# (tools/build.sh is deliberately not listed: a size-gate or message edit must
# not bump the version; an FQBN change always lands with a source change, and
# release.sh's same-version guard diffs build.sh as the backstop.)
FIRMWARE_PATHS = 'AutoFOV_V12.ino AutoFOV_V12_wifi.ino data/index.html data/FreeSans7pt7b.h tools/partitions.csv build_opt.h'
# ─────────────────────────────────────────────────────────────────────────────

root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
src  = os.path.join(root, 'data', 'index.html')
dst  = os.path.join(root, 'data', 'web_ui.h')

# Git info
def git(cmd):
    try:
        return subprocess.check_output(cmd, cwd=root, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return '?'

count = git(['git', 'rev-list', '--count', 'HEAD', '--', *FIRMWARE_PATHS.split()])
commit = git(['git', 'rev-parse', '--short', 'HEAD'])
try:
    patch = max(0, int(count) - VERSION_PATCH_BASE)
except ValueError:
    patch = count  # git unavailable — leave whatever git() returned

# Uncommitted work → label the build with the version it is ABOUT TO BECOME.
# The patch number is derived from the commit count, so a build.sh run on a
# dirty tree used to stamp the PREVIOUS release's number: you flash a .bin
# named v12.5.33 that is not the v12.5.33 that shipped, which defeats the whole
# point of stamping a version. The next real commit makes this patch+1, so use
# that and suffix the commit hash with '+' — a dev build can then never be
# confused with the release of the same number.
# release.sh still refuses to PUBLISH a dirty tree (it must stamp a version that
# corresponds to a real commit); this only makes local test builds honest. Same
# Only FIRMWARE_PATHS count (same rule as the commit count): a dirty tooling
# file must not relabel the build, and web_ui.h is not listed — it is this
# script's own output.
_status = git(['git', 'status', '--porcelain', '--', *FIRMWARE_PATHS.split()])
if _status not in ('?', ''):
    _dirty = [ln for ln in _status.split('\n') if ln.strip()]
    if _dirty and isinstance(patch, int):
        patch += 1
        commit = f'{commit}+'

version = f'{VERSION_MAJOR}.{VERSION_MINOR}.{patch}'

# Source metrics — counted across the project's hand-written files so the
# About screen reports an accurate, build-fresh figure.  web_ui.h is the
# generated artifact (would double-count index.html) and is excluded.
SOURCE_FILES = [
    'AutoFOV_V12.ino', 'AutoFOV_V12_wifi.ino', 'build_opt.h',
    'data/index.html', 'data/FreeSans7pt7b.h',
    'tools/build.sh', 'tools/drift-check.sh',
    'tools/flash_firmware_only.sh', 'tools/release.sh',
    'tools/embed_html.py',
]
sloc  = 0
sbytes = 0
for rel in SOURCE_FILES:
    full = os.path.join(root, rel)
    try:
        with open(full, 'rb') as fp:
            data = fp.read()
            sbytes += len(data)
            sloc   += data.count(b'\n') + (1 if data and not data.endswith(b'\n') else 0)
    except OSError:
        pass
skb = (sbytes + 512) // 1024     # round to nearest KB

def node_check(node, js):
    """`node --check` a script body. Returns (rc, stderr_text). node refuses
    files without a .js extension, hence the named temp file."""
    with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False) as fp:
        fp.write(js)
        path = fp.name
    try:
        r = subprocess.run([node, '--check', path], capture_output=True, text=True)
    finally:
        os.unlink(path)
    return r.returncode, r.stderr


def minify_inline(html):
    """Shrink the inline <script>/<style> blocks before gzip.

    Gate FIRST, minify SECOND: the ORIGINAL <script> bodies must pass
    `node --check`, or this script exits 1 and build.sh (set -e) stops before
    compiling — a dashboard with a syntax error must never reach the binary.
    (Previously only the MINIFIED copy was checked and a failure fell back to
    embedding the original — which was the broken source itself, unminified,
    with exit 0: a broken dashboard shipped ~100 KB larger than usual.)

    Minifying is comment/whitespace removal only (rjsmin/rcssmin never rename
    identifiers). It stays fail-safe: a missing library, an exception, or a
    minified body that fails node --check when the original passed (minifier
    bug) falls back to the original body with a LOUD stderr warning — the
    worst case is a larger binary, never a broken dashboard. data/index.html
    stays the human-readable source; only the embedded copy is minified.
    Returns (html, note)."""
    import sys
    node = shutil.which('node')
    if not node:
        sys.stderr.write('WARNING: node not found — dashboard JS is UNVERIFIED '
                         '(install node to gate builds on `node --check`)\n')

    # Stash each <script> body (replace with a sentinel) so the CSS/comment
    # passes can never touch a "<style>" or "<!--" that is really a JS string.
    scripts = []
    def stash(m):
        attrs, body = m.group(1), m.group(2)
        t = re.search(r'type\s*=\s*["\']([^"\']+)["\']', attrs or '', re.I)
        if t and t.group(1).lower() not in (
                'text/javascript', 'application/javascript', 'module'):
            return m.group(0)          # leave JSON / template scripts alone
        scripts.append((attrs, body))
        return f'\x00S{len(scripts) - 1}\x00'
    tmp = re.sub(r'<script\b([^>]*)>(.*?)</script>', stash, html,
                 flags=re.S | re.I)

    # 1. Gate the ORIGINAL source. A failure here is a build error, full stop.
    if node:
        for i, (_, body) in enumerate(scripts):
            rc, err = node_check(node, body)
            if rc != 0:
                sys.stderr.write(
                    f'ERROR: data/index.html <script> #{i + 1} fails node --check '
                    f'— fix the source before building:\n{err}\n')
                sys.exit(1)

    # 2. Minify JS (fail-safe per block).
    try:
        import rjsmin
    except ImportError:
        rjsmin = None
    note = 'min on' if node else 'min on (unverified — node not found)'
    if rjsmin is None:
        note = 'min off — pip3 install rjsmin rcssmin to enable'
    minified = []
    for i, (attrs, body) in enumerate(scripts):
        out = body
        if rjsmin is not None:
            try:
                cand = rjsmin.jsmin(body)
                if node:
                    rc, err = node_check(node, cand)
                    if rc != 0:
                        sys.stderr.write(
                            f'WARNING: minified <script> #{i + 1} fails node --check '
                            f'but the ORIGINAL passes (minifier bug?) — embedding the '
                            f'original, UN-minified:\n{err}\n')
                        note = 'min off — minified JS failed node --check (original embedded)'
                    else:
                        out = cand
                else:
                    out = cand
            except Exception as e:
                sys.stderr.write(f'WARNING: JS minify failed ({e}) — embedding original\n')
                note = f'min off — {e} (original embedded)'
        minified.append((attrs, out))

    # 3. CSS (comment/whitespace only) and HTML structural comments — both run
    #    while scripts are still sentinels, so JS is untouched.
    try:
        import rcssmin
        tmp = re.sub(
            r'<style\b([^>]*)>(.*?)</style>',
            lambda m: f'<style{m.group(1)}>{rcssmin.cssmin(m.group(2))}</style>',
            tmp, flags=re.S | re.I)
    except Exception as e:
        sys.stderr.write(f'WARNING: CSS minify skipped ({e})\n')
    tmp = re.sub(r'<!--.*?-->', '', tmp, flags=re.S)

    out = re.sub(
        r'\x00S(\d+)\x00',
        lambda m: '<script{0}>{1}</script>'.format(*minified[int(m.group(1))]),
        tmp)
    return out, note

with open(src, 'rb') as f:
    raw = f.read()

payload, mnote = minify_inline(raw.decode('utf-8'))
payload = payload.encode('utf-8')
compressed = gzip.compress(payload, compresslevel=9)

out = []
out.append('// Auto-generated by tools/embed_html.py — do not edit manually.')
out.append('// Re-run the script whenever data/index.html changes, then recompile.')
out.append('#pragma once')
out.append('#include <pgmspace.h>')
out.append('')
out.append(f'#define BUILD_VERSION "{version}"')
out.append(f'#define BUILD_COMMIT  "{commit}"')
out.append(f'#define BUILD_SLOC    {sloc}')
out.append(f'#define BUILD_SKB     {skb}')
out.append('')
out.append(f'// {len(raw)} bytes raw  ->  {len(payload)} bytes minified  ->  {len(compressed)} bytes gzip')
out.append('static const uint8_t WEB_UI_HTML_GZ[] PROGMEM = {')
for i in range(0, len(compressed), 16):
    chunk = compressed[i:i+16]
    out.append('    ' + ', '.join(f'0x{b:02x}' for b in chunk) + ',')
out.append('};')
out.append(f'static const size_t WEB_UI_HTML_GZ_LEN = {len(compressed)};')

with open(dst, 'w') as f:
    f.write('\n'.join(out) + '\n')

print(f'v{version} ({commit})  |  html: {len(raw):,} -> {len(payload):,} min -> {len(compressed):,} gz [{mnote}]  |  sloc: {sloc:,} ({skb} KB)')
