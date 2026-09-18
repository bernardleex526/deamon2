#!/bin/bash
# ============================================================================
# build_cloud_export.sh — NEW-09 build script (compile only; never runs the
# binary, not even --help; the main agent owns all executions).
#
# - Resolves every path from this script's own location (workspace-derived,
#   never $HOME-defaulted): bin path = <workspace>/bin/drdds_cloud_export.
# - mkdir only creates the target bin directory; NO rm / cleanup anywhere.
# - Links ONLY the vendor SDK stack (/usr/local): libdrdds, libfastrtps 2.14,
#   libfastcdr, pthread — plus an rpath so the loaded libdrdds matches the
#   NEW-07-proven /usr/local/lib/libdrdds.so.1.1.7.
# - Does NOT link any ROS/Foxy library and does not touch /opt/ros.
# - No chmod on system paths: g++ output is executable as-is.
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
BIN_DIR="$WS_ROOT/bin"
BIN_PATH="$BIN_DIR/drdds_cloud_export"
SRC_PATH="$SCRIPT_DIR/drdds_cloud_export.cpp"

mkdir -p "$BIN_DIR"

g++ -std=c++17 -O2 -Wall -Wextra \
    -I/usr/local/include -I/usr/local/include/dridl \
    -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
    -o "$BIN_PATH" "$SRC_PATH" \
    -ldrdds -lfastrtps -lfastcdr -lpthread

echo "built: $BIN_PATH"
