#!/bin/sh
# Install git hooks from .githooks/ into .git/hooks/
# Run once after cloning: sh install-hooks.sh

REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
if [ -z "$REPO_ROOT" ]; then
    echo "Error: not inside a git repository."
    exit 1
fi

HOOKS_SRC="$REPO_ROOT/.githooks"
HOOKS_DST="$REPO_ROOT/.git/hooks"

for hook in "$HOOKS_SRC"/*; do
    name=$(basename "$hook")
    dst="$HOOKS_DST/$name"
    cp "$hook" "$dst"
    chmod +x "$dst"
    echo "Installed: $name"
done

echo "Done. All git hooks installed."

