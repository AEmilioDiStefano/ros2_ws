#!/usr/bin/env python3

# reset_gz.py

# THIS TOOL enables the reset_gz command which wipes Gazebo / Ignition GUI 
# config and caches for the current user

# USE THIS TO GET RID OF ROBOTS THAT SHOULD NOT APPEAR IN OUR SIM
# (this can happen when a removed xacro still exists somewhere on the system)

import os
import shutil
from pathlib import Path
from typing import List


# Paths we want to remove
TARGETS: List[Path] = [
    Path("~/.config/ignition").expanduser(),
    Path("~/.config/gz").expanduser(),
    Path("~/.gz").expanduser(),
    Path("~/.ignition").expanduser(),
    Path("~/.gazebo").expanduser(),
]


def _remove_path(path: Path) -> None:
    """
    Remove a file or directory if it exists, ignoring errors.
    """
    if not path.exists():
        return

    if path.is_dir():
        shutil.rmtree(path, ignore_errors=True)
    else:
        try:
            path.unlink(missing_ok=True)  # Python 3.8+
        except TypeError:
            # For older Python versions
            try:
                path.unlink()
            except FileNotFoundError:
                pass


def main() -> None:
    print("This will remove Gazebo / Ignition GUI config and cache for the current user.")
    print("It will delete (if they exist):")
    for p in TARGETS:
        print(f"  - {p}")
    print()

    ans = input("Continue? [y/N] ").strip().lower()

    if ans not in ("y", "yes"):
        print("Aborted. No changes made.")
        return

    print("\nClearing Gazebo / Ignition state...")

    for p in TARGETS:
        if p.exists():
            print(f"  Removing {p}")
            _remove_path(p)
        else:
            print(f"  Skipping {p} (does not exist)")

    print("\nDone. Restart Gazebo / your ROS launch and re-test.")


if __name__ == "__main__":
    main()
