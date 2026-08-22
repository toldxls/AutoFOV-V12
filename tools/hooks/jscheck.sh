#!/bin/bash
# Shared by pre-commit and pre-push: `jscheck_html <file>` extracts every
# <script> body from an HTML file and runs `node --check` on the lot.
# Syntax-only — node ignores the undefined browser globals (document,
# WebSocket, …), just the grammar. Prints node's error (indented) on failure.
# Returns 0 = ok, 1 = syntax error, 0 (fail-open) when node is absent.
jscheck_html() {
    local page="$1"
    command -v node >/dev/null 2>&1 || return 0
    # Temp DIR with a .js file inside — node --check rejects files whose
    # extension it doesn't recognise, and mktemp can't guarantee a .js suffix.
    local tmpd js err
    tmpd=$(mktemp -d -t autofov_js.XXXXXX) || return 0
    js="$tmpd/check.js"; err="$tmpd/err.txt"
    # File argument, not stdin — piping into `python3 - <<PY` makes the pipe
    # and the heredoc fight over stdin (shell-dependent who wins).
    python3 - "$page" >"$js" <<'PY'
import re, sys
html = open(sys.argv[1], encoding='utf-8').read()
blocks = re.findall(r'<script\b[^>]*>(.*?)</script>', html, flags=re.S | re.I)
sys.stdout.write('\n;\n'.join(blocks))
PY
    if ! node --check "$js" 2>"$err"; then
        sed 's/^/  /' "$err" >&2
        rm -rf "$tmpd"
        return 1
    fi
    rm -rf "$tmpd"
    return 0
}
