#!/usr/bin/env python3
"""Generate web_ui.h from data/index.html.
Run once after any change to index.html (or to bump VERSION_MINOR),
then recompile and OTA-flash the firmware.
Usage: python3 tools/embed_html.py
"""
import gzip, os, re, shutil, subprocess, tempfile

# ── Version ───────────────────────────────────────────────────────────────────
VERSION_MAJOR = 12
VERSION_MINOR = 2   # bump manually for milestone releases
# Patch counter resets to 0 at each minor bump. Set VERSION_PATCH_BASE to the
# commit count at the bump commit so patch = HEAD_count - base.  Without this,
# each new series would inherit the old running count instead of starting at
# .0.  JS semver compare orders by minor first, so the OTA update gate sees
# 12.2.x > 12.1.y regardless of patch.
# 12.2: the bug-sweep series — calibration sync, vib inStack latch, OTA reboot
# overlay, auth reconnect, TOF signal smoothing.
VERSION_PATCH_BASE = 168
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

count  = git(['git', 'rev-list', '--count', 'HEAD'])
commit = git(['git', 'rev-parse', '--short', 'HEAD'])
try:
    patch = max(0, int(count) - VERSION_PATCH_BASE)
except ValueError:
    patch = count  # git unavailable — leave whatever git() returned
version = f'{VERSION_MAJOR}.{VERSION_MINOR}.{patch}'

# Source metrics — counted across the project's hand-written files so the
# About screen reports an accurate, build-fresh figure.  web_ui.h is the
# generated artifact (would double-count index.html) and is excluded.
SOURCE_FILES = [
    'AutoFOV_V12.ino', 'AutoFOV_V12_wifi.ino',
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

def minify_inline(html):
    """Shrink the inline <script>/<style> blocks before gzip — comment and
    whitespace removal only (rjsmin/rcssmin are non-destructive: they never
    rename identifiers, so semantics are preserved). Fully fail-safe: on a
    missing library, an exception, or minified JS that does not pass
    `node --check`, the ORIGINAL html is returned untouched. The worst case is
    a slightly larger binary — never a broken build or a broken dashboard.
    data/index.html stays the human-readable source; only the embedded copy is
    minified. Returns (html, note)."""
    try:
        import rjsmin, rcssmin
    except ImportError:
        return html, 'min off — pip3 install rjsmin rcssmin to enable'

    try:
        # Stash each <script> body first (replace with a sentinel) so the CSS
        # pass can never touch a "<style>" that is really a JS string literal.
        scripts = []
        def stash(m):
            attrs, body = m.group(1), m.group(2)
            t = re.search(r'type\s*=\s*["\']([^"\']+)["\']', attrs or '', re.I)
            if t and t.group(1).lower() not in (
                    'text/javascript', 'application/javascript', 'module'):
                return m.group(0)          # leave JSON / template scripts alone
            scripts.append((attrs, rjsmin.jsmin(body)))
            return f'\x00S{len(scripts) - 1}\x00'
        tmp = re.sub(r'<script\b([^>]*)>(.*?)</script>', stash, html,
                     flags=re.S | re.I)
        tmp = re.sub(
            r'<style\b([^>]*)>(.*?)</style>',
            lambda m: f'<style{m.group(1)}>{rcssmin.cssmin(m.group(2))}</style>',
            tmp, flags=re.S | re.I)
        # Drop HTML structural comments (<!-- ... -->) — inert on the device.
        # Runs while scripts are still stashed as sentinels, so JS is untouched;
        # CSS uses /* */ so it has none of these either.
        tmp = re.sub(r'<!--.*?-->', '', tmp, flags=re.S)

        # Parse-gate every minified script. node is the developer-machine
        # default; if it is absent we still minify (rjsmin is safe) but say so.
        node = shutil.which('node')
        if node:
            for _, js in scripts:
                with tempfile.NamedTemporaryFile(
                        'w', suffix='.js', delete=False) as fp:
                    fp.write(js)
                    path = fp.name
                try:
                    rc = subprocess.run([node, '--check', path],
                                        capture_output=True).returncode
                finally:
                    os.unlink(path)
                if rc != 0:
                    return html, 'min off — node --check failed (using original)'

        out = re.sub(
            r'\x00S(\d+)\x00',
            lambda m: '<script{0}>{1}</script>'.format(*scripts[int(m.group(1))]),
            tmp)
        return out, ('min on' if node else 'min on (unverified — node not found)')
    except Exception as e:
        return html, f'min off — {e} (using original)'


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
