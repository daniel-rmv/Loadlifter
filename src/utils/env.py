#!/usr/bin/env python3
# env.py
# Lightweight helpers for loading .env files and expanding ${VAR} placeholders.
# Author: Daniel Würmli

import os
import re
from pathlib import Path
from typing import Any, Union

_ENV_ASSIGN_RE = re.compile(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)\s*$")
_ENV_PLACEHOLDER_RE = re.compile(r"\$\{([A-Z0-9_]+)(?::([^}]*))?\}")


class MissingEnvValueError(ValueError):
    """Raised when a required environment variable placeholder has no value."""


def _default_dotenv_path() -> Path:
    """Return the default .env location at the project root."""
    override = os.environ.get("LOADLIFTER_ENV_FILE")
    if override:
        return Path(override).expanduser()
    return Path(__file__).resolve().parents[2] / ".env"


def load_dotenv(dotenv_path: Union[str, Path, None] = None, override: bool = False) -> Path:
    """
    Load key=value pairs from a dotenv file into the environment.

    :param dotenv_path: Optional explicit path. Defaults to project-root/.env.
    :param override: When True, existing environment values are overwritten.
    :return: Path to the dotenv file (even if missing) to aid debugging.
    """
    path = Path(dotenv_path) if dotenv_path else _default_dotenv_path()
    if not path.exists():
        return path

    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            match = _ENV_ASSIGN_RE.match(line)
            if not match:
                continue  # Skip malformed lines silently
            key, value = match.groups()
            # Remove surrounding quotes if present
            if value and (value[0] == value[-1]) and value[0] in "\"'":
                value = value[1:-1]
            if override or key not in os.environ:
                os.environ[key] = value
    return path


def _expand_placeholder(match: re.Match) -> str:
    var, default = match.group(1), match.group(2)
    if var in os.environ:
        return os.environ[var]
    if default is not None:
        return default
    raise MissingEnvValueError(f"Environment variable '{var}' is required but not set.")


def expand_env_placeholders(value: Any) -> Any:
    """
    Recursively expand ${VAR[:default]} placeholders in strings, lists and mappings.
    """
    if isinstance(value, str):
        return _ENV_PLACEHOLDER_RE.sub(_expand_placeholder, value)
    if isinstance(value, list):
        return [expand_env_placeholders(item) for item in value]
    if isinstance(value, tuple):
        return tuple(expand_env_placeholders(item) for item in value)
    if isinstance(value, set):
        return {expand_env_placeholders(item) for item in value}
    if isinstance(value, dict):
        return {
            key: expand_env_placeholders(item)
            for key, item in value.items()
        }
    return value


__all__ = ["load_dotenv", "expand_env_placeholders", "MissingEnvValueError"]
