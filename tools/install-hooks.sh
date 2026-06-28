#!/bin/bash
# Point git at the version-controlled hooks in tools/hooks/ for this repo.
# Run once per clone:  bash tools/install-hooks.sh
set -e
cd "$(dirname "$0")/.."
chmod +x tools/hooks/* 2>/dev/null || true
git config core.hooksPath tools/hooks
echo "core.hooksPath -> tools/hooks   (active hooks: $(ls tools/hooks | tr '\n' ' '))"
