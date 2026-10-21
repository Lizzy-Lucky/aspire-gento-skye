#!/usr/bin/env bash
# Copy the Gento Skye files that live outside this repository into gento_skye/, so they are
# versioned together with the ASPIRE changes. Run before every commit:
#
#     bash gento_skye/sync_from_workspace.sh
#
# The 469 MB USD and the BEHAVIOR dataset are not copied (see gento_skye/README.md).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${GENTO_WORKSPACE:-/root/gpufree-data/aspire}"
MODEL="$WS/behavior-data/omnigibson-robot-assets/models/gento_skye"

copy_tree() {  # src dst [names to drop after copying...]
  local src="$1" dst="$2"
  shift 2
  rm -rf "$dst"
  mkdir -p "$dst"
  cp -a "$src/." "$dst/"
  local name
  for name in "$@"; do
    find "$dst" -name "$name" -prune -exec rm -rf {} +
  done
}

copy_tree "$WS/gento-skye-import" "$HERE/import" __pycache__ import.log
copy_tree "$MODEL" "$HERE/robot_assets/gento_skye" usd __pycache__
copy_tree "$WS/gento_skye_description" "$HERE/description/gento_skye_description"

echo "synced into $HERE:"
du -sh "$HERE/import" "$HERE/robot_assets" "$HERE/description"
