#!/usr/bin/env python3
"""
CubeMX post-codegen script.

Steps:
  1. CRLF → LF  – all files tracked by git (respects .gitignore automatically).
  2. clang-format -i  – *.c / *.h / *.cpp / *.hpp under Core/.

Usage – set as CubeMX "Post-run" script:
  python scripts/post_codegen.py
  python3 scripts/post_codegen.py

CubeMX invokes the script with cwd = project root (where the .ioc file is).
"""

import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Directories to run clang-format on (relative to project root)
FORMAT_DIRS = ["Core"]

# Extensions eligible for clang-format
FORMAT_EXTENSIONS = {".c", ".h", ".cpp", ".hpp"}

# clang-format executable (must be on PATH)
CLANG_FORMAT = "clang-format"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def git_tracked_files(root: Path) -> list[Path]:
    """
    Return all files currently tracked by git (i.e. not ignored, not untracked).
    Uses  git ls-files  so .gitignore is honoured automatically.
    Also includes files that are not yet staged but exist on disk via
    `git ls-files --others --exclude-standard` is intentionally excluded –
    we only want committed / staged files to stay consistent.
    """
    try:
        result = subprocess.run(
            ["git", "ls-files"],
            cwd=str(root),
            capture_output=True,
            text=True,
            check=True,
        )
    except FileNotFoundError:
        print("[ERROR] 'git' not found. Make sure git is installed and on PATH.",
              file=sys.stderr)
        return []
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] git ls-files failed:\n{exc.stderr.strip()}", file=sys.stderr)
        return []

    paths = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if line:
            p = root / line
            if p.is_file():
                paths.append(p)
    return paths


def convert_crlf_to_lf(path: Path) -> bool:
    """Replace CRLF with LF in *path*. Returns True if the file was changed."""
    try:
        raw = path.read_bytes()
    except OSError as exc:
        print(f"  [WARN] Cannot read {path}: {exc}", file=sys.stderr)
        return False

    if b"\r\n" not in raw:
        return False

    path.write_bytes(raw.replace(b"\r\n", b"\n"))
    return True


def run_clang_format(path: Path) -> bool:
    """Run clang-format -i on *path*. Returns True on success."""
    try:
        result = subprocess.run(
            [CLANG_FORMAT, "-i", str(path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"  [WARN] clang-format failed for {path}:\n{result.stderr.strip()}",
                  file=sys.stderr)
            return False
        return True
    except FileNotFoundError:
        print(
            f"[ERROR] '{CLANG_FORMAT}' not found. "
            "Make sure clang-format is installed and available on PATH.",
            file=sys.stderr,
        )
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    root = Path.cwd()
    print(f"post_codegen: project root = {root}")

    # ── Step 1: CRLF → LF for all git-tracked files ─────────────────────────
    tracked = git_tracked_files(root)
    if not tracked:
        print("[WARN] No git-tracked files found – skipping CRLF conversion.")
    else:
        print(f"\n[1/2] CRLF → LF: scanning {len(tracked)} git-tracked file(s)...")
        crlf_count = 0
        for f in sorted(tracked):
            if convert_crlf_to_lf(f):
                crlf_count += 1
                print(f"  [CRLF→LF] {f.relative_to(root)}")
        if crlf_count == 0:
            print("  (no files needed conversion)")
        else:
            print(f"  Fixed {crlf_count} file(s).")

    # ── Step 2: clang-format for Core/**/*.{c,h,cpp,hpp} ────────────────────
    format_files: list[Path] = []
    for rel_dir in FORMAT_DIRS:
        target = root / rel_dir
        if not target.is_dir():
            print(f"  [WARN] FORMAT_DIR not found, skipping: {rel_dir}", file=sys.stderr)
            continue
        for p in target.rglob("*"):
            if p.is_file() and p.suffix in FORMAT_EXTENSIONS:
                format_files.append(p)

    if not format_files:
        print("\n[2/2] clang-format: no source files found – skipping.")
    else:
        print(f"\n[2/2] clang-format: processing {len(format_files)} file(s)...")
        fmt_ok = fmt_fail = 0
        for f in sorted(format_files):
            if run_clang_format(f):
                fmt_ok += 1
            else:
                fmt_fail += 1
        print(f"  ok: {fmt_ok}, failed: {fmt_fail}.")

    print("\npost_codegen: all done.")
    return 1 if (format_files and fmt_fail) else 0


if __name__ == "__main__":
    sys.exit(main())

