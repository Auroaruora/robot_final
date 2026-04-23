#!/usr/bin/env bash
# Launcher for run_robot.py — sets DYLD_LIBRARY_PATH so the Vicon C SDK
# dylibs can load, then invokes Python. Forwards all args.

set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VICON_LIB="$DIR/vicon-py-sdk/vicon_sdk"

if [[ ! -d "$VICON_LIB" ]]; then
  echo "warning: Vicon SDK dir not found at $VICON_LIB" >&2
fi

export DYLD_LIBRARY_PATH="${VICON_LIB}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"

exec python3 "$DIR/run_robot.py" "$@"
