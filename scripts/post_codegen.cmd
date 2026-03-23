: # -*- mode: sh -*-
: # Polyglot launcher: valid .cmd (Windows) AND valid .sh (Linux/macOS).
: # CubeMX UAScriptAfterPath calls this after code generation.
: #
: # On Windows:  cmd.exe executes the @goto block, runs python.
: # On Unix:     sh ignores ":" lines, runs the shell block below.
: #
: # CubeMX sets cwd to its own install directory (not the project),
: # so we derive the project root from this script's location: tools/ -> ..

:<<"::CMDLITERAL"
@goto :CMDSCRIPT
::CMDLITERAL

# ── Unix path (sh / bash) ──────────────────────────────────────────────────
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

if command -v python3 >/dev/null 2>&1; then
    PYTHON=python3
elif command -v python >/dev/null 2>&1; then
    PYTHON=python
else
    echo "[post_codegen] ERROR: python not found" >&2
    exit 1
fi

echo "[post_codegen] Using $PYTHON (project: $PROJECT_ROOT)"
exec "$PYTHON" "$SCRIPT_DIR/post_codegen.py"

:CMDSCRIPT
@echo off
setlocal

:: cd to project root (tools\..\) — CubeMX may set cwd to its install dir
cd /d "%~dp0.."

:: Try 'python' first (real install on Windows), then 'python3' (Linux/macOS alias).
:: On Windows, 'python3' is often a broken Windows Store shim — try it last.
python --version >nul 2>&1 && (
    echo [post_codegen] Using python
    python "%~dp0post_codegen.py"
    exit /b %ERRORLEVEL%
)

where python3 >nul 2>&1 && (
    echo [post_codegen] Using python3
    python3 "%~dp0post_codegen.py"
    exit /b %ERRORLEVEL%
)

echo [post_codegen] ERROR: python not found >&2
exit /b 1
