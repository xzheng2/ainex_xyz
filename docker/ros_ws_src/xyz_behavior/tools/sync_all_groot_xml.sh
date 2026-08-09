#!/usr/bin/env bash
# Sync every *groot.xml under xyz_behavior from its sibling bt.py.
# Produces a *groot.synced.xml next to each original. Non-destructive.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XYZ_BEHAVIOR="$(cd "$DIR/.." && pwd)"
TOOL="$DIR/sync_groot_xml.py"

files=0; changes=0; warnings=0
while IFS= read -r -d '' xml; do
    files=$((files + 1))
    out="$(python3 "$TOOL" "$xml" || true)"
    [ -n "$out" ] && printf '%s\n' "$out"
    changes=$((changes + $(printf '%s\n' "$out" | grep -cE '^  [^ ].*: .* -> ' || true)))
    warnings=$((warnings + $(printf '%s\n' "$out" | grep -cE '^  WARN ' || true)))
done < <(find "$XYZ_BEHAVIOR" -name '*groot.xml' ! -name '*.synced.xml' -print0 | sort -z)

echo "----------------------------------------------------------------"
echo "Done: scanned $files file(s), $changes change(s), $warnings warning(s)."
