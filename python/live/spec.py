"""The one spec file: limits and battery the Pi monitor runs with.

The planner writes it for a sized drone; the monitor (and the simulation) read
it, so the numbers on the Pi are exactly the ones the design was checked with.
"""

from __future__ import annotations

import json
from dataclasses import asdict, fields
from pathlib import Path

from droneguard.thresholds import Thresholds

VERSION = 1


def spec_dict(c: Thresholds, **meta) -> dict:
    return {"droneguard_spec": VERSION, "thresholds": asdict(c), **meta}


def save_spec(path: str | Path, c: Thresholds, **meta) -> None:
    Path(path).write_text(json.dumps(spec_dict(c, **meta), indent=2))


def load_spec(path: str | Path) -> Thresholds:
    d = json.loads(Path(path).read_text())
    if d.get("droneguard_spec") != VERSION:
        raise ValueError(f"{path} is not a DroneGuard spec (version {VERSION})")
    known = {f.name for f in fields(Thresholds)}
    return Thresholds(**{k: v for k, v in d["thresholds"].items() if k in known})
