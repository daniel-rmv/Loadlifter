#!/usr/bin/env python3
# __main__.py
# Allow `python -m src` to launch the CLI entrypoint.
# Author: Daniel Würmli

"""Allow `python -m src` to launch the CLI entrypoint."""

from .cli import main


if __name__ == "__main__":
    main()
