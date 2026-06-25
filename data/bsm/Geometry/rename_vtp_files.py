#!/usr/bin/env python3
"""
Dry-run and apply renamer for files with duplicated extensions like '.vtp.vtp' or '.vtp.ply'.

Usage:
  python rename_vtp_files.py [--apply]

Default directory is the script's containing folder.
Without --apply the script only prints planned renames (safe dry-run).
"""
from pathlib import Path
import sys

def rename_vtp_like_files(dirpath: Path, apply: bool = False):
    if not dirpath.exists() or not dirpath.is_dir():
        print(f"ERROR: directory not found: {dirpath}")
        return 2

    changed = 0
    for p in sorted(dirpath.iterdir()):
        if not p.is_file():
            continue
        name = p.name
        # Only consider files that contain ".vtp." (e.g. .vtp.vtp, .vtp.ply, .vtp.xyz)
        if ".vtp." not in name:
            continue

        # Collapse any suffix after the first '.vtp.' to a single '.vtp'
        # e.g. 'cervical_spine_alone.vtp.vtp' -> 'cervical_spine_alone.vtp'
        new_name = name.split('.vtp.', 1)[0] + '.vtp'
        new_path = p.with_name(new_name)
        if new_path.exists():
            print(f"SKIP (target exists): {p.name} -> {new_path.name}")
            continue
        if apply:
            p.rename(new_path)
            print(f"RENAMED: {p.name} -> {new_path.name}")
        else:
            print(f"DRY-RUN: {p.name} -> {new_path.name}")
        changed += 1

    if changed == 0:
        print("No matching files found.")
    else:
        print(f"Processed {changed} file(s). {'(applied)' if apply else '(dry-run)'}")
    return 0

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true', help='Actually perform renames (default is dry-run)')
    args = ap.parse_args()
    script_dir = Path(__file__).resolve().parent
    sys.exit(rename_vtp_like_files(script_dir, apply=args.apply))
