#!/usr/bin/env python3
# __init__.py
# Package marker for robot control.
# Author: Daniel Würmli

"""
Loadlifter control package.

Automatically loads environment variables from a local .env file when the
package is imported so that secrets and machine-specific paths stay out of the
tracked configuration.
"""

from .utils.env import load_dotenv

# Load .env once at import. Missing files are ignored intentionally.
load_dotenv()

