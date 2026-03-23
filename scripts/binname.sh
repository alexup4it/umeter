#!/bin/sh

# Resolve project root relative to this script's location
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PARAMS_FILE="$PROJECT_ROOT/Core/Inc/params.h"

DEVICE_NAME=$(grep PARAMS_DEVICE_NAME "$PARAMS_FILE" | awk '{print $3}')
DEVICE_VER1=$(grep PARAMS_FW_B1 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER2=$(grep PARAMS_FW_B2 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER3=$(grep PARAMS_FW_B3 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER4=$(grep PARAMS_FW_B4 "$PARAMS_FILE" | head -n1 | awk '{print $3}')

BIN="${DEVICE_NAME//\"}-${DEVICE_VER1}.${DEVICE_VER2}.${DEVICE_VER3}.${DEVICE_VER4}.bin"

cp "${1}.bin" "$BIN"
