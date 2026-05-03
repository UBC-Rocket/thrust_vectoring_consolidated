from __future__ import annotations

import random
import ctypes
import hashlib
import math
import subprocess
import tempfile
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Iterable, Sequence


def _coerce_vec3(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {len(values)}")

    return (float(values[0]), float(values[1]), float(values[2]))


def _distance_between(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


class Tag:
    pos: tuple[float, float, float]
    distance: float | None = None

    def __init__(self, pos: Sequence[float], distance: float | None = None, errorFunction=(lambda: 0)):
        self.truePos = pos
        self.pos = (pos[0] + errorFunction(), pos[1] + errorFunction(), pos[2] + errorFunction())

        if distance is not None:
            distance = float(distance)
        self.distance = distance
        
    def set_error_function(self, errorFunction):
        self.pos = (self.pos[0] + errorFunction(), self.pos[1] + errorFunction(), self.pos[2] + errorFunction())



class Simulation:
    def __init__(
        self,
        tags: Iterable[Tag],
        target_pos: Sequence[float] | None = None,
        distance_function=_distance_between,
        error_function=(lambda: 0)
    ) -> None:
        self.tags = list(tags)
        self.num_tags = len(self.tags)
        self.target_pos = None if target_pos is None else _coerce_vec3(target_pos)
        self.distance_function=distance_function

        for tag in self.tags:
            tag.set_error_function(error_function)


class _CTag(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_float * 3),
        ("distance", ctypes.c_float),
    ]


@lru_cache(maxsize=1)
def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _shared_library_path() -> Path:
    repo_root = _repo_root()
    source = repo_root / "src" / "trilat.c"
    header = repo_root / "include" / "trilat.h"
    cache_key = hashlib.sha256(
        source.read_bytes() + header.read_bytes()
    ).hexdigest()[:16]

    build_dir = Path(tempfile.gettempdir()) / "trilateration_sim"
    build_dir.mkdir(parents=True, exist_ok=True)
    return build_dir / f"libtrilateration_{cache_key}.so"


def _build_shared_library() -> Path:
    repo_root = _repo_root()
    source = repo_root / "src" / "trilat.c"
    include_dir = repo_root / "include"
    library_path = _shared_library_path()

    if library_path.exists():
        return library_path

    compile_command = [
        "cc",
        "-shared",
        "-fPIC",
        "-O2",
        str(source),
        "-I",
        str(include_dir),
        "-lm",
        "-o",
        str(library_path),
    ]

    try:
        subprocess.run(
            compile_command,
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            "Failed to build the trilateration shared library.\n"
            f"Command: {' '.join(compile_command)}\n"
            f"stderr:\n{exc.stderr}"
        ) from exc

    return library_path


@lru_cache(maxsize=1)
def _load_solver():
    library = ctypes.CDLL(str(_build_shared_library()))
    solver = library.trilaterate_n_tags
    solver.argtypes = [
        ctypes.POINTER(_CTag),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_float),
    ]
    solver.restype = ctypes.c_int
    return solver


def _resolve_tag_distance(tag: Tag, sim: Simulation) -> float:
    if tag.distance is not None:
        return tag.distance

    if sim.target_pos is None:
        raise ValueError(
            "Each tag needs a distance, or Simulation.target_pos must be set."
        )

    return sim.distance_function(tag.truePos, sim.target_pos)


def run_c_code(sim: Simulation) -> tuple[float, float, float]:
    if sim.num_tags < 4:
        raise ValueError("trilaterate_n_tags requires at least 4 tags.")

    c_tags = (_CTag * sim.num_tags)()
    for index, tag in enumerate(sim.tags):
        distance = _resolve_tag_distance(tag, sim)
        c_tags[index] = _CTag(
            (ctypes.c_float * 3)(*tag.pos),
            ctypes.c_float(distance),
        )

    output_pos = (ctypes.c_float * 3)()
    ok = _load_solver()(c_tags, sim.num_tags, output_pos)
    if ok != 1:
        raise RuntimeError(
            "trilaterate_n_tags failed. Check that the tag geometry is valid."
        )

    return tuple(float(output_pos[axis]) for axis in range(3))

def wrong_distance_function(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )

if __name__ == "__main__":

    # Calibration call

    sim = Simulation(
        tags=[
            Tag((0.0, 0.0, 0.0)),
            Tag((2.0, 0.0, 0.0)),
            Tag((0.0, 4.0, 0.0)),
            Tag((0.0, 0.0, 14.0)),
        ],
        target_pos=(1.0, 1.0, 1.0),
        distance_function=_distance_between
    )

    print("Estimated position, Calibration:", run_c_code(sim))

    # Error calls

    sim = Simulation(
        tags=[
            Tag((0.0, 0.0, 0.0)),
            Tag((2.0, 0.0, 0.0)),
            Tag((0.0, 4.0, 0.0)),
            Tag((0.0, 0.0, 14.0)),
        ],
        target_pos=(1.0, 1.0, 1.0),
        distance_function=wrong_distance_function,
        error_function=(lambda: random.uniform(-10, 10))
    )

    print("Off estimated position:", run_c_code(sim))
