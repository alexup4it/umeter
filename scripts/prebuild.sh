#!/bin/sh

# Resolve project root relative to this script's location
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

git log --pretty=format:"#define GIT_COMMIT_HASH \"%H\"%n" -n 1 > "$PROJECT_ROOT/Core/Inc/gitcommit.h"
