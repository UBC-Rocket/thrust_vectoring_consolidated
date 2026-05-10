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

# some bullshit type guarentee bullshit
def _coerce_vec3(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {len(values)}")

    return (float(values[0]), float(values[1]), float(values[2]))

# no error introducecs
def _distance_between(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )

# represents a tag with a position and a recorded distance to the rocket
class Tag:
    pos: tuple[float, float, float]

    def __init__(self, pos: Sequence[float], distance: float | None = None, errorFunction=(lambda: 0)):
        self.truePos = pos
        self.pos = (pos[0], pos[1], pos[2])
        self.errorFunction = errorFunction

        if distance is not None:
            distance = float(distance)
        self.distance = distance
        
    def set_error_function(self, errorFunction):
        self.errorFunction = errorFunction

    def get_position(self) -> tuple[float, float, float]:
        return self.pos
    
    def get_distance(
        self,
        comparison: Sequence[float] | None = None,
        distance_function=_distance_between,
    ) -> float:
        if self.distance is not None:
            return self.distance

        if comparison is None:
            raise ValueError("A comparison position is required when no distance is set.")

        noisy_pos = (
            self.pos[0] + self.errorFunction(),
            self.pos[1] + self.errorFunction(),
            self.pos[2] + self.errorFunction(),
        )

        return distance_function(noisy_pos, comparison)
        



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
    try:
        return tag.get_distance(sim.target_pos, sim.distance_function)
    except ValueError as exc:
        raise ValueError(
            "Each tag needs a distance, or Simulation.target_pos must be set."
        ) from exc


def run_c_code(sim: Simulation) -> tuple[float, float, float]:
    if sim.num_tags < 4:
        raise ValueError("trilaterate_n_tags requires at least 4 tags.")

    c_tags = (_CTag * sim.num_tags)()
    for index, tag in enumerate(sim.tags):
        distance = _resolve_tag_distance(tag, sim)
        c_tags[index] = _CTag(
            (ctypes.c_float * 3)(*tag.get_position()),
            ctypes.c_float(distance),
        )

    output_pos = (ctypes.c_float * 3)()
    ok = _load_solver()(c_tags, sim.num_tags, output_pos)
    if ok != 1:
        raise RuntimeError(
            "trilaterate_n_tags failed. Check that the tag geometry is valid."
        )

    return tuple(float(output_pos[axis]) for axis in range(3))

def generate_point(xy_range=100.0, z_range=1.0):
    return (
        random.uniform(-xy_range, xy_range),
        random.uniform(-xy_range, xy_range),
        random.uniform(-z_range, z_range),
    )

def run_num_times(
    error,
    num_runs,
    num_tags=4,
    xy_range=100.0,
    z_range=1.0,
    return_components=False,
):
    axis_sums = [0.0, 0.0, 0.0]
    completed_runs = 0
    attempts = 0
    max_attempts = num_runs * 10

    while completed_runs < num_runs and attempts < max_attempts:
        attempts += 1
        sim = Simulation(
            tags=[
                Tag(generate_point(xy_range, z_range)) for k in range(num_tags)
            ],
            target_pos=generate_point(xy_range, z_range),
            distance_function=_distance_between,
            error_function=(lambda: random.uniform(-error, error))
        )

        try:
            ret = run_c_code(sim)
        except RuntimeError:
            continue

        delta = [abs(ret[j] - sim.target_pos[j]) for j in range(3)]
        for j in range(3):
            axis_sums[j] += delta[j]
        completed_runs += 1

    if completed_runs < num_runs:
        raise RuntimeError(
            f"Only completed {completed_runs} valid trilateration runs "
            f"after {attempts} attempts."
        )
    
    axis_averages = [axis_sum / completed_runs for axis_sum in axis_sums]
    xyz_average = sum(axis_averages) / 3
    xy_average = (axis_averages[0] + axis_averages[1]) / 2

    if return_components:
        return {
            "xyz": xyz_average,
            "xy": xy_average,
            "z": axis_averages[2],
            "x": axis_averages[0],
            "y": axis_averages[1],
        }

    return xyz_average

def sweep_error_response(
    min_error=0.0,
    max_error=30.0,
    steps=25,
    runs_per_step=1000,
    num_tags=7,
    unit="cm",
    meters_per_unit=0.01,
    xy_range=100.0,
    z_range=1.0,
):
    if steps < 2:
        raise ValueError("steps must be at least 2.")

    results = []
    for i in range(steps):
        input_error = min_error + (max_error - min_error) * i / (steps - 1)
        output_errors_m = run_num_times(
            input_error * meters_per_unit,
            runs_per_step,
            num_tags,
            xy_range,
            z_range,
            return_components=True,
        )
        output_errors = {
            key: value / meters_per_unit
            for key, value in output_errors_m.items()
        }
        results.append((input_error, output_errors))
        print(
            f"tag error {input_error:.3f} {unit} -> "
            f"xyz {output_errors['xyz']:.6f} {unit}, "
            f"xy {output_errors['xy']:.6f} {unit}, "
            f"z {output_errors['z']:.6f} {unit}"
        )

    return results

def write_error_graph_svg(results, output_path, unit="m", geometry_note=""):
    width = 900
    height = 560
    margin_left = 80
    margin_right = 30
    margin_top = 52
    margin_bottom = 75
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    max_x = max(point[0] for point in results)
    series = [
        ("xyz", "Average X/Y/Z", "#0077b6"),
        ("xy", "Horizontal X/Y", "#2a9d8f"),
        ("z", "Vertical Z", "#d1495b"),
    ]
    max_y = max(point[1][key] for point in results for key, _, _ in series)
    max_x = max(max_x, 1.0)
    max_y = max(max_y, 1.0e-9)

    def x_pos(value):
        return margin_left + (value / max_x) * plot_width

    def y_pos(value):
        return margin_top + plot_height - (value / max_y) * plot_height

    series_nodes = []
    legend_nodes = []
    for series_index, (key, label, color) in enumerate(series):
        points = " ".join(
            f"{x_pos(input_error):.2f},{y_pos(output_errors[key]):.2f}"
            for input_error, output_errors in results
        )
        point_nodes = "\n".join(
            f'<circle cx="{x_pos(input_error):.2f}" cy="{y_pos(output_errors[key]):.2f}" r="3.5" fill="{color}" />'
            for input_error, output_errors in results
        )
        series_nodes.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3" />\n'
            f'{point_nodes}'
        )
        legend_y = margin_top + 20 + series_index * 22
        legend_nodes.append(
            f'<line x1="{width - 250}" y1="{legend_y}" x2="{width - 225}" y2="{legend_y}" stroke="{color}" stroke-width="3" />'
            f'<text x="{width - 215}" y="{legend_y + 5}">{label}</text>'
        )

    x_ticks = []
    y_ticks = []
    for tick in range(6):
        x_value = max_x * tick / 5
        x = x_pos(x_value)
        x_ticks.append(
            f'<line x1="{x:.2f}" y1="{margin_top + plot_height}" '
            f'x2="{x:.2f}" y2="{margin_top + plot_height + 6}" stroke="#222" />'
            f'<text x="{x:.2f}" y="{height - 42}" text-anchor="middle">{x_value:.1f}</text>'
        )

        y_value = max_y * tick / 5
        y = y_pos(y_value)
        y_ticks.append(
            f'<line x1="{margin_left - 6}" y1="{y:.2f}" '
            f'x2="{margin_left}" y2="{y:.2f}" stroke="#222" />'
            f'<line x1="{margin_left}" y1="{y:.2f}" '
            f'x2="{margin_left + plot_width}" y2="{y:.2f}" stroke="#e6e6e6" />'
            f'<text x="{margin_left - 12}" y="{y + 4:.2f}" text-anchor="end">{y_value:.3f}</text>'
        )

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#ffffff" />
<style>
text {{ font-family: Arial, sans-serif; font-size: 14px; fill: #222; }}
.title {{ font-size: 22px; font-weight: 700; }}
.note {{ font-size: 13px; fill: #555; }}
.label {{ font-size: 16px; font-weight: 600; }}
</style>
<text x="{width / 2}" y="24" text-anchor="middle" class="title">Tag Measurement Error vs Output Position Error ({unit})</text>
<text x="{width / 2}" y="43" text-anchor="middle" class="note">{geometry_note}</text>
{''.join(y_ticks)}
<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_height}" stroke="#222" stroke-width="2" />
<line x1="{margin_left}" y1="{margin_top + plot_height}" x2="{margin_left + plot_width}" y2="{margin_top + plot_height}" stroke="#222" stroke-width="2" />
{''.join(x_ticks)}
{''.join(series_nodes)}
{''.join(legend_nodes)}
<text x="{margin_left + plot_width / 2}" y="{height - 12}" text-anchor="middle" class="label">Max independent tag coordinate error ({unit})</text>
<text x="22" y="{margin_top + plot_height / 2}" text-anchor="middle" class="label" transform="rotate(-90 22 {margin_top + plot_height / 2})">Output position error ({unit})</text>
</svg>
'''

    output_path = Path(output_path)
    output_path.write_text(svg)
    return output_path

def graph_error_response(
    min_error=0.0,
    max_error=30.0,
    steps=25,
    runs_per_step=1000,
    num_tags=7,
    unit="cm",
    meters_per_unit=0.01,
    xy_range=100.0,
    z_range=1.0,
    output_path=None,
):
    if output_path is None:
        output_path = Path(__file__).with_name("tag_error_vs_output_error.svg")

    results = sweep_error_response(
        min_error,
        max_error,
        steps,
        runs_per_step,
        num_tags,
        unit,
        meters_per_unit,
        xy_range,
        z_range,
    )
    geometry_note = f"Generated geometry: X/Y range +/-{xy_range:g} m, Z range +/-{z_range:g} m"
    return write_error_graph_svg(results, output_path, unit, geometry_note)

def run_num_times_coord_repeats(
    error,
    num_runs,
    num_tags=4,
    coord_repeats=100,
    progress=False,
    xy_range=100.0,
    z_range=1.0,
):
    coord_map = []

    for i in range(num_runs):
        if progress:
            print(i, "/", num_runs)

        delta_sum = 0
        tags = [Tag(generate_point(xy_range, z_range)) for k in range(num_tags)]
        target_pos = (0.0, 0.0, 0.0)

        sim = Simulation(
            tags=tags,
            target_pos=target_pos,
            distance_function=_distance_between,
            error_function=(lambda: random.uniform(-error, error))
        )

        valid_repeats = 0
        for j in range(coord_repeats):
            try:
                ret = run_c_code(sim)
            except RuntimeError:
                continue

            delta = [abs(ret[j] - sim.target_pos[j]) for j in range(3)]
            for j in range(3):
                delta_sum += delta[j]
            valid_repeats += 1
        
        if valid_repeats == 0:
            continue

        avg_delta = delta_sum / (valid_repeats * 3)

        coord_map.append([avg_delta, tags, target_pos])

    return sorted(coord_map, key=(lambda x: x[0]))

def write_candidate_distribution_svg(candidates, output_path, top_n=5, unit="m"):
    top_candidates = candidates[:top_n]
    panel_width = 900
    panel_height = 255
    width = panel_width
    height = 55 + panel_height * len(top_candidates)
    top_size = 190
    side_width = 360
    side_height = 90

    all_points = []
    for _, tags, target_pos in top_candidates:
        all_points.extend(tag.get_position() for tag in tags)
        all_points.append(target_pos)

    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    min_z = min(point[2] for point in all_points)
    max_z = max(point[2] for point in all_points)

    def padded_bounds(min_value, max_value, fallback_span=1.0):
        span = max_value - min_value
        if span < 1.0e-9:
            span = fallback_span
            center = (min_value + max_value) / 2
            min_value = center - span / 2
            max_value = center + span / 2
        padding = span * 0.08
        return min_value - padding, max_value + padding

    min_x, max_x = padded_bounds(min_x, max_x)
    min_y, max_y = padded_bounds(min_y, max_y)
    min_z, max_z = padded_bounds(min_z, max_z)

    def map_x(value, left, size):
        return left + (value - min_x) / (max_x - min_x) * size

    def map_y(value, top, size):
        return top + size - (value - min_y) / (max_y - min_y) * size

    def map_z(value, top, size):
        return top + size - (value - min_z) / (max_z - min_z) * size

    panels = []
    for index, (avg_delta, tags, target_pos) in enumerate(top_candidates):
        panel_top = 50 + index * panel_height
        top_left = 45
        top_top = panel_top + 42
        side_left = 315
        side_top = panel_top + 96
        avg_delta_cm = avg_delta * 100

        tag_points_xy = []
        tag_points_xz = []
        tag_labels = []
        for tag_index, tag in enumerate(tags, start=1):
            position = tag.get_position()
            x = map_x(position[0], top_left, top_size)
            y = map_y(position[1], top_top, top_size)
            sx = map_x(position[0], side_left, side_width)
            sz = map_z(position[2], side_top, side_height)
            tag_points_xy.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="#0077b6" />')
            tag_points_xz.append(f'<circle cx="{sx:.2f}" cy="{sz:.2f}" r="5" fill="#0077b6" />')
            tag_labels.append(f'<text x="{x + 8:.2f}" y="{y + 4:.2f}">T{tag_index}</text>')

        target_x = map_x(target_pos[0], top_left, top_size)
        target_y = map_y(target_pos[1], top_top, top_size)
        target_sx = map_x(target_pos[0], side_left, side_width)
        target_sz = map_z(target_pos[2], side_top, side_height)

        panels.append(f'''
<g>
<text x="45" y="{panel_top + 20}" class="panel-title">Candidate {index + 1}: average output error {avg_delta_cm:.2f} cm</text>
<rect x="{top_left}" y="{top_top}" width="{top_size}" height="{top_size}" fill="#fafafa" stroke="#333" />
<text x="{top_left + top_size / 2}" y="{top_top - 10}" text-anchor="middle" class="label">Top view: X/Y ({unit})</text>
{''.join(tag_points_xy)}
<circle cx="{target_x:.2f}" cy="{target_y:.2f}" r="7" fill="#d1495b" />
<path d="M {target_x - 8:.2f} {target_y:.2f} L {target_x + 8:.2f} {target_y:.2f} M {target_x:.2f} {target_y - 8:.2f} L {target_x:.2f} {target_y + 8:.2f}" stroke="#ffffff" stroke-width="2" />
{''.join(tag_labels)}
<rect x="{side_left}" y="{side_top}" width="{side_width}" height="{side_height}" fill="#fafafa" stroke="#333" />
<text x="{side_left + side_width / 2}" y="{side_top - 10}" text-anchor="middle" class="label">Side view: X/Z ({unit})</text>
{''.join(tag_points_xz)}
<circle cx="{target_sx:.2f}" cy="{target_sz:.2f}" r="7" fill="#d1495b" />
<path d="M {target_sx - 8:.2f} {target_sz:.2f} L {target_sx + 8:.2f} {target_sz:.2f} M {target_sx:.2f} {target_sz - 8:.2f} L {target_sx:.2f} {target_sz + 8:.2f}" stroke="#ffffff" stroke-width="2" />
<text x="{side_left}" y="{side_top + side_height + 28}">Blue: tags. Red: target. Ranked from repeated noisy measurement solves.</text>
</g>
''')

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#ffffff" />
<style>
text {{ font-family: Arial, sans-serif; font-size: 13px; fill: #222; }}
.title {{ font-size: 22px; font-weight: 700; }}
.panel-title {{ font-size: 17px; font-weight: 700; }}
.label {{ font-size: 14px; font-weight: 600; }}
</style>
<text x="{width / 2}" y="28" text-anchor="middle" class="title">Top Tag Point Distribution Candidates</text>
{''.join(panels)}
</svg>
'''

    output_path = Path(output_path)
    output_path.write_text(svg)
    return output_path

def graph_top_distribution_candidates(
    error_cm=10.0,
    num_layouts=150,
    num_tags=7,
    coord_repeats=150,
    top_n=5,
    xy_range=100.0,
    z_range=1.0,
    output_path=None,
):
    if output_path is None:
        output_path = Path(__file__).with_name("top_point_distribution_candidates.svg")

    candidates = run_num_times_coord_repeats(
        error_cm * 0.01,
        num_layouts,
        num_tags,
        coord_repeats,
        progress=False,
        xy_range=xy_range,
        z_range=z_range,
    )
    return write_candidate_distribution_svg(candidates, output_path, top_n)


if __name__ == "__main__":
    graph_path = graph_error_response(min_error=0.0, max_error=30.0, steps=50, runs_per_step=1000, num_tags=10, unit="cm", meters_per_unit=0.01, z_range=10.0)
    print(f"Saved error graph to: {graph_path}")

    # graph_path = graph_top_distribution_candidates(
    #     error_cm=10.0,
    #     num_layouts=1500,
    #     num_tags=7,
    #     coord_repeats=1500,
    #     top_n=5,
    # )
    # print(f"Saved candidate distribution diagrams to: {graph_path}")



  
