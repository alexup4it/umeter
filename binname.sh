#!/bin/sh

# Usage: binname.sh <binary_name> [project_root]
# If project_root is not provided, falls back to ../

PROJECT_ROOT="${2:-.}"
PARAMS_FILE="${PROJECT_ROOT}/Core/Inc/params.h"

DEVICE_NAME=$(grep PARAMS_DEVICE_NAME "$PARAMS_FILE" | awk '{print $3}')
DEVICE_VER1=$(grep PARAMS_FW_B1 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER2=$(grep PARAMS_FW_B2 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER3=$(grep PARAMS_FW_B3 "$PARAMS_FILE" | head -n1 | awk '{print $3}')
DEVICE_VER4=$(grep PARAMS_FW_B4 "$PARAMS_FILE" | head -n1 | awk '{print $3}')

BIN="${DEVICE_NAME//\"}-${DEVICE_VER1}.${DEVICE_VER2}.${DEVICE_VER3}.${DEVICE_VER4}.bin"

cp "${1}.bin" "$BIN"
