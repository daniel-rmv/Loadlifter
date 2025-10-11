#!/usr/bin/env python3
# update_loc_badge.py
# update the loc badge in README.md
# Author: Daniel Würmli
from __future__ import annotations

import itertools
from pathlib import Path


TARGET_DIRS = [Path("src"), Path("tests"), Path("scripts")]
BADGE_PATH = Path("docs/badges/python_loc.svg")
LABEL = "Python LOC"


def iter_python_files(paths: list[Path]) -> itertools.chain[Path]:
    """Yield all Python files under the provided directories."""
    return itertools.chain.from_iterable(
        path.rglob("*.py") for path in paths if path.exists()
    )


def count_lines(files: itertools.chain[Path]) -> int:
    """Count total lines across the given files."""
    total = 0
    for file_path in files:
        with file_path.open("rb") as handle:
            total += sum(1 for _ in handle)
    return total


def text_width(text: str) -> int:
    """Approximate the pixel width of badge text."""
    return len(text) * 6 + 14


def build_svg(label: str, value: str) -> str:
    """Render a simple shield-style SVG badge."""
    left_width = text_width(label)
    right_width = text_width(value)
    total_width = left_width + right_width
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{total_width}" height="20" role="img" aria-label="{label}: {value}">
  <title>{label}: {value}</title>
  <linearGradient id="locBadgeGradient" x2="0" y2="100%">
    <stop offset="0" stop-color="#bbb" stop-opacity=".1" />
    <stop offset="1" stop-opacity=".1" />
  </linearGradient>
  <mask id="locBadgeMask">
    <rect width="{total_width}" height="20" rx="3" fill="#fff" />
  </mask>
  <g mask="url(#locBadgeMask)">
    <rect width="{left_width}" height="20" fill="#555" />
    <rect x="{left_width}" width="{right_width}" height="20" fill="#007ec6" />
    <rect width="{total_width}" height="20" fill="url(#locBadgeGradient)" />
  </g>
  <g fill="#fff" text-anchor="middle" font-family="DejaVu Sans,Verdana,Geneva,sans-serif" font-size="11">
    <text x="{left_width / 2}" y="15" fill="#010101" fill-opacity=".3">{label}</text>
    <text x="{left_width / 2}" y="14">{label}</text>
    <text x="{left_width + right_width / 2}" y="15" fill="#010101" fill-opacity=".3">{value}</text>
    <text x="{left_width + right_width / 2}" y="14">{value}</text>
  </g>
</svg>
"""


def main() -> None:
    files = list(iter_python_files(TARGET_DIRS))
    count = count_lines(files)
    BADGE_PATH.parent.mkdir(parents=True, exist_ok=True)
    BADGE_PATH.write_text(build_svg(LABEL, str(count)), encoding="utf-8")
    print(f"LOC badge updated: {count} lines -> {BADGE_PATH}")


if __name__ == "__main__":
    main()
