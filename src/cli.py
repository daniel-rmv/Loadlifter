#!/usr/bin/env python3
# cli.py
# CLI entry point bridging arguments to the control system.
# Author: Daniel Würmli

"""CLI entry point bridging arguments to the control system."""

from .control.control_system import main as control_main


def main():
    """Dispatch to the legacy control-system CLI."""
    control_main()


if __name__ == "__main__":
    main()
