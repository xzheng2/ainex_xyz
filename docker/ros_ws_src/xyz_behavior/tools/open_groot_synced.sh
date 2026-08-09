#!/usr/bin/env bash
# Sync the given project *groot.xml from its bt.py, then open Groot on the
# synced file.  Usage: ./open_groot_synced.sh path/to/project/xxx_groot.xml
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOL="$DIR/sync_groot_xml.py"
GROOT="${GROOT_LAUNCHER:-/home/pi/groot.sh}"

if [ $# -ne 1 ]; then
    echo "usage: $(basename "$0") path/to/xxx_groot.xml" >&2
    exit 2
fi

xml="$1"
case "$xml" in
    *groot.xml) ;;
    *) echo "error: expected a *groot.xml file, got: $xml" >&2; exit 2 ;;
esac
if [ ! -f "$xml" ]; then
    echo "error: no such file: $xml" >&2
    exit 2
fi

python3 "$TOOL" "$xml"
synced="${xml%.xml}.synced.xml"
if [ ! -f "$synced" ]; then
    echo "error: sync did not produce $synced" >&2
    exit 1
fi

echo "Opening Groot on $synced"
exec "$GROOT" "$synced"
