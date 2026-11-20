"""Katalog / servo / referans geometri yükleyicileri (varsayılan yol: data/)."""
import json
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def _load(path, default_name):
    p = Path(path) if path is not None else DATA_DIR / default_name
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def load_actuators(path=None):
    return _load(path, "actuators.json")


def load_servos(path=None):
    return _load(path, "servos.json")


def load_reference_geometry(path=None):
    return _load(path, "reference_geometry.json")
