#!/usr/bin/env python3
"""
Trajectory and yaw-coupling simulation for tilt_kf.c.

This script compiles the real C implementation into a temporary shared library,
drives tilt_kf_update() with simulated IMU data, and compares the estimate
against either a generated two-axis tilt profile or a fixed-lean yaw spin.

Default output:
    - printed tracking metrics
    - CSV at /tmp/tilt_kf_sim.csv

Examples:
    python3 sim_tilt_kf.py
    python3 sim_tilt_kf.py --plot
    python3 sim_tilt_kf.py --duration 20 --rate-hz 500 --linear-accel-g 0.2
    python3 sim_tilt_kf.py --scenario yaw-cone --yaw-rate-deg-s 180 \
        --accel-noise-g 0 --gyro-noise-rad-s 0 \
        --accel-bias-g 0,0,0 --gyro-bias-rad-s 0,0,0 --linear-accel-g 0
"""

from __future__ import annotations

import argparse
import ctypes
import csv
import math
import random
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


GRAV_MPS2 = 9.80665
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


@dataclass(frozen=True)
class ImuModel:
    accel_noise_g: float
    gyro_noise_rad_s: float
    accel_bias_g: tuple[float, float, float]
    gyro_bias_rad_s: tuple[float, float, float]
    linear_accel_g: float
    timestamp_jitter_us: float


@dataclass
class SimSample:
    t_s: float
    true_yaw_deg: float
    true_x_deg: float
    true_y_deg: float
    raw_x_deg: float
    raw_y_deg: float
    est_x_deg: float
    est_y_deg: float
    true_rate_x_rad_s: float
    true_rate_y_rad_s: float
    ax_mps2: float
    ay_mps2: float
    az_mps2: float
    gx_rad_s: float
    gy_rad_s: float
    gz_rad_s: float
    kf_bias_x_rad_s: float
    kf_bias_y_rad_s: float
    horiz_accel_g: float
    upright: bool


class TiltKf1D(ctypes.Structure):
    _fields_ = [
        ("angle", ctypes.c_float),
        ("bias", ctypes.c_float),
        ("P", (ctypes.c_float * 2) * 2),
        ("last_t_us", ctypes.c_uint64),
        ("initialised", ctypes.c_bool),
    ]


class TiltKf(ctypes.Structure):
    _fields_ = [
        ("x", TiltKf1D),
        ("y", TiltKf1D),
        ("horiz_accel_g", ctypes.c_float),
    ]


class TiltKfC:
    def __init__(self, lib_path: Path):
        self.lib = ctypes.CDLL(str(lib_path))

        self.lib.tilt_kf_init.argtypes = [ctypes.POINTER(TiltKf)]
        self.lib.tilt_kf_init.restype = None

        self.lib.tilt_kf_update.argtypes = [
            ctypes.POINTER(TiltKf),
            ctypes.c_uint64,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self.lib.tilt_kf_update.restype = None

        self.lib.tilt_kf_angle_x_deg.argtypes = [ctypes.POINTER(TiltKf)]
        self.lib.tilt_kf_angle_x_deg.restype = ctypes.c_float

        self.lib.tilt_kf_angle_y_deg.argtypes = [ctypes.POINTER(TiltKf)]
        self.lib.tilt_kf_angle_y_deg.restype = ctypes.c_float

        self.lib.tilt_kf_is_upright.argtypes = [
            ctypes.POINTER(TiltKf),
            ctypes.c_float,
        ]
        self.lib.tilt_kf_is_upright.restype = ctypes.c_bool

    def new_filter(self) -> TiltKf:
        kf = TiltKf()
        self.lib.tilt_kf_init(ctypes.byref(kf))
        return kf

    def update(
        self,
        kf: TiltKf,
        t_us: int,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
    ) -> None:
        self.lib.tilt_kf_update(
            ctypes.byref(kf),
            ctypes.c_uint64(t_us),
            ctypes.c_float(ax),
            ctypes.c_float(ay),
            ctypes.c_float(az),
            ctypes.c_float(gx),
            ctypes.c_float(gy),
            ctypes.c_float(gz),
        )

    def angle_x_deg(self, kf: TiltKf) -> float:
        return float(self.lib.tilt_kf_angle_x_deg(ctypes.byref(kf)))

    def angle_y_deg(self, kf: TiltKf) -> float:
        return float(self.lib.tilt_kf_angle_y_deg(ctypes.byref(kf)))

    def is_upright(self, kf: TiltKf, threshold_deg: float = 5.0) -> bool:
        return bool(
            self.lib.tilt_kf_is_upright(ctypes.byref(kf), ctypes.c_float(threshold_deg))
        )


def state_estimation_root() -> Path:
    return Path(__file__).resolve().parents[1]


def build_c_filter(source_root: Path, build_dir: Path) -> Path:
    src = source_root / "src" / "tilt_kf.c"
    include = source_root / "include"
    lib_name = "libtilt_kf.dylib" if sys.platform == "darwin" else "libtilt_kf.so"
    lib_path = build_dir / lib_name
    build_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "gcc",
        "-shared",
        "-fPIC",
        "-std=c99",
        "-Wall",
        "-Wextra",
        f"-I{include}",
        str(src),
        "-lm",
        "-o",
        str(lib_path),
    ]
    result = subprocess.run(cmd, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(
            "could not build tilt_kf.c\n"
            f"command: {' '.join(cmd)}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

    return lib_path


def half_cosine(a0: float, a1: float, u: float) -> float:
    s = 0.5 - 0.5 * math.cos(math.pi * u)
    return a0 + (a1 - a0) * s


def base_profile_deg(t_s: float, duration_s: float) -> tuple[float, float]:
    """Piecewise smooth commanded tilt profile in degrees."""
    keyframes = [
        (0.00, 0.0, 0.0),
        (0.08, 0.0, 0.0),
        (0.25, 18.0, 0.0),
        (0.38, 18.0, -10.0),
        (0.55, -12.0, 8.0),
        (0.70, 0.0, 12.0),
        (0.82, 10.0, -6.0),
        (0.94, 0.0, 0.0),
        (1.00, 0.0, 0.0),
    ]
    q = min(max(t_s / duration_s, 0.0), 1.0)

    for i in range(len(keyframes) - 1):
        q0, x0, y0 = keyframes[i]
        q1, x1, y1 = keyframes[i + 1]
        if q <= q1:
            u = 0.0 if q1 == q0 else (q - q0) / (q1 - q0)
            return half_cosine(x0, x1, u), half_cosine(y0, y1, u)

    _, x, y = keyframes[-1]
    return x, y


def profile_deg(t_s: float, duration_s: float) -> tuple[float, float]:
    """Truth tilt profile with a small vibration burst during the middle."""
    x_deg, y_deg = base_profile_deg(t_s, duration_s)
    q = min(max(t_s / duration_s, 0.0), 1.0)

    burst_start = 0.55
    burst_end = 0.82
    if burst_start <= q <= burst_end:
        u = (q - burst_start) / (burst_end - burst_start)
        envelope = math.sin(math.pi * u) ** 2
        x_deg += 2.5 * envelope * math.sin(2.0 * math.pi * 2.2 * t_s)
        y_deg += 1.5 * envelope * math.sin(2.0 * math.pi * 1.7 * t_s + 0.3)

    return x_deg, y_deg


def truth_state(t_s: float, duration_s: float) -> tuple[float, float, float, float]:
    """Return truth tilt x/y and rates x/y in radians and radians/sec."""
    t_s = min(max(t_s, 0.0), duration_s)
    x_deg, y_deg = profile_deg(t_s, duration_s)

    eps = min(1e-4, duration_s * 1e-5)
    t0 = max(0.0, t_s - eps)
    t1 = min(duration_s, t_s + eps)
    if t1 == t0:
        rate_x_deg_s = 0.0
        rate_y_deg_s = 0.0
    else:
        x0, y0 = profile_deg(t0, duration_s)
        x1, y1 = profile_deg(t1, duration_s)
        rate_x_deg_s = (x1 - x0) / (t1 - t0)
        rate_y_deg_s = (y1 - y0) / (t1 - t0)

    return (
        x_deg * DEG2RAD,
        y_deg * DEG2RAD,
        rate_x_deg_s * DEG2RAD,
        rate_y_deg_s * DEG2RAD,
    )


def gravity_accel_from_tilts(
    x_rad: float,
    y_rad: float,
    g_mps2: float = GRAV_MPS2,
) -> tuple[float, float, float]:
    """
    Create a gravity-only accelerometer vector matching tilt_kf.c measurements.

    tilt_kf.c measures:
        tilt_x = atan2(ay, ax)
        tilt_y = atan2(az, ax)
    """
    tx = math.tan(x_rad)
    ty = math.tan(y_rad)
    ax = g_mps2 / math.sqrt(1.0 + tx * tx + ty * ty)
    ay = ax * tx
    az = ax * ty
    return ax, ay, az


def yaw_cone_state(
    t_s: float,
    tilt_rad: float,
    yaw_rate_rad_s: float,
) -> tuple[float, float, float, float, float, float, float, float]:
    """
    Return a rigid-body-consistent fixed-lean rotation about sensor x.

    Sensor x maps to body-down in tilt_kf.h, so this is a yaw spin. The
    transverse gravity projection rotates between sensor y/z while the gyro
    reports gx only. This is the coupling that two independent tilt filters
    cannot predict from gy/gz.
    """
    yaw_rad = yaw_rate_rad_s * t_s
    axial = GRAV_MPS2 * math.cos(tilt_rad)
    transverse = GRAV_MPS2 * math.sin(tilt_rad)
    ay = transverse * math.cos(yaw_rad)
    az = -transverse * math.sin(yaw_rad)

    tilt_x = math.atan2(ay, axial)
    tilt_y = math.atan2(az, axial)

    # Exact derivatives of the two atan2 tilt measurements. They differ from
    # gy/gz because their change is caused entirely by gx cross-axis coupling.
    day_dt = -transverse * yaw_rate_rad_s * math.sin(yaw_rad)
    daz_dt = -transverse * yaw_rate_rad_s * math.cos(yaw_rad)
    rate_x = axial * day_dt / (axial * axial + ay * ay)
    rate_y = axial * daz_dt / (axial * axial + az * az)

    return tilt_x, tilt_y, rate_x, rate_y, yaw_rad, axial, ay, az


def linear_accel_disturbance(
    t_s: float,
    duration_s: float,
    amplitude_g: float,
) -> tuple[float, float, float]:
    """Non-gravity acceleration that intentionally contaminates accel tilt."""
    if amplitude_g <= 0.0:
        return 0.0, 0.0, 0.0

    start = 0.43 * duration_s
    end = 0.60 * duration_s
    if t_s < start or t_s > end:
        return 0.0, 0.0, 0.0

    u = (t_s - start) / (end - start)
    envelope = math.sin(math.pi * u) ** 2
    ay = amplitude_g * GRAV_MPS2 * envelope
    az = 0.45 * amplitude_g * GRAV_MPS2 * envelope * math.sin(2.0 * math.pi * 1.2 * t_s)
    return 0.0, ay, az


def simulate(
    api: TiltKfC,
    duration_s: float,
    rate_hz: float,
    model: ImuModel,
    seed: int,
    scenario: str,
    yaw_tilt_deg: float,
    yaw_rate_deg_s: float,
) -> list[SimSample]:
    rng = random.Random(seed)
    kf = api.new_filter()
    samples: list[SimSample] = []

    nominal_step_us = 1_000_000.0 / rate_hz
    total_steps = int(math.ceil(duration_s * rate_hz))
    t_us = 0

    for i in range(total_steps + 1):
        t_s = min(t_us * 1e-6, duration_s)
        if scenario == "yaw-cone":
            (
                true_x,
                true_y,
                true_rate_x,
                true_rate_y,
                true_yaw,
                ax,
                ay,
                az,
            ) = yaw_cone_state(
                t_s,
                yaw_tilt_deg * DEG2RAD,
                yaw_rate_deg_s * DEG2RAD,
            )
            true_gx = yaw_rate_deg_s * DEG2RAD
            true_gy = 0.0
            true_gz = 0.0
        else:
            true_x, true_y, true_rate_x, true_rate_y = truth_state(t_s, duration_s)
            true_yaw = 0.0
            ax, ay, az = gravity_accel_from_tilts(true_x, true_y)
            true_gx = 0.0
            true_gy = true_rate_y
            true_gz = true_rate_x

        dax, day, daz = linear_accel_disturbance(t_s, duration_s, model.linear_accel_g)
        ax += dax
        ay += day
        az += daz

        ax += model.accel_bias_g[0] * GRAV_MPS2 + rng.gauss(0.0, model.accel_noise_g * GRAV_MPS2)
        ay += model.accel_bias_g[1] * GRAV_MPS2 + rng.gauss(0.0, model.accel_noise_g * GRAV_MPS2)
        az += model.accel_bias_g[2] * GRAV_MPS2 + rng.gauss(0.0, model.accel_noise_g * GRAV_MPS2)

        gx = true_gx + model.gyro_bias_rad_s[0] + rng.gauss(0.0, model.gyro_noise_rad_s)
        gy = true_gy + model.gyro_bias_rad_s[1] + rng.gauss(0.0, model.gyro_noise_rad_s)
        gz = true_gz + model.gyro_bias_rad_s[2] + rng.gauss(0.0, model.gyro_noise_rad_s)

        api.update(kf, t_us, ax, ay, az, gx, gy, gz)

        samples.append(
            SimSample(
                t_s=t_s,
                true_yaw_deg=true_yaw * RAD2DEG,
                true_x_deg=true_x * RAD2DEG,
                true_y_deg=true_y * RAD2DEG,
                raw_x_deg=math.atan2(ay, ax) * RAD2DEG,
                raw_y_deg=math.atan2(az, ax) * RAD2DEG,
                est_x_deg=api.angle_x_deg(kf),
                est_y_deg=api.angle_y_deg(kf),
                true_rate_x_rad_s=true_rate_x,
                true_rate_y_rad_s=true_rate_y,
                ax_mps2=ax,
                ay_mps2=ay,
                az_mps2=az,
                gx_rad_s=gx,
                gy_rad_s=gy,
                gz_rad_s=gz,
                kf_bias_x_rad_s=float(kf.x.bias),
                kf_bias_y_rad_s=float(kf.y.bias),
                horiz_accel_g=float(kf.horiz_accel_g),
                upright=api.is_upright(kf),
            )
        )

        if i < total_steps:
            jitter = rng.uniform(-model.timestamp_jitter_us, model.timestamp_jitter_us)
            step_us = max(1, int(round(nominal_step_us + jitter)))
            t_us += step_us

    return samples


def rms(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values))


def wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def tilt_azimuth_error_deg(sample: SimSample) -> float:
    truth_azimuth = math.atan2(sample.true_y_deg, sample.true_x_deg) * RAD2DEG
    estimate_azimuth = math.atan2(sample.est_y_deg, sample.est_x_deg) * RAD2DEG
    return wrap_deg(estimate_azimuth - truth_azimuth)


def metrics(samples: list[SimSample], skip_s: float) -> dict[str, float]:
    use = [s for s in samples if s.t_s >= skip_s]
    if not use:
        use = samples

    est_x_err = [s.est_x_deg - s.true_x_deg for s in use]
    est_y_err = [s.est_y_deg - s.true_y_deg for s in use]
    raw_x_err = [s.raw_x_deg - s.true_x_deg for s in use]
    raw_y_err = [s.raw_y_deg - s.true_y_deg for s in use]
    direction_samples = [
        s for s in use if math.hypot(s.true_x_deg, s.true_y_deg) >= 1.0
    ]
    azimuth_err = [tilt_azimuth_error_deg(s) for s in direction_samples]
    magnitude_err = [
        math.hypot(s.est_x_deg, s.est_y_deg)
        - math.hypot(s.true_x_deg, s.true_y_deg)
        for s in direction_samples
    ]

    return {
        "est_x_rmse": rms(est_x_err),
        "est_y_rmse": rms(est_y_err),
        "raw_x_rmse": rms(raw_x_err),
        "raw_y_rmse": rms(raw_y_err),
        "est_x_max": max(abs(e) for e in est_x_err),
        "est_y_max": max(abs(e) for e in est_y_err),
        "raw_x_max": max(abs(e) for e in raw_x_err),
        "raw_y_max": max(abs(e) for e in raw_y_err),
        "tilt_azimuth_mean": (
            sum(azimuth_err) / len(azimuth_err) if azimuth_err else math.nan
        ),
        "tilt_azimuth_rmse": rms(azimuth_err) if azimuth_err else math.nan,
        "tilt_azimuth_max": (
            max(abs(e) for e in azimuth_err) if azimuth_err else math.nan
        ),
        "tilt_magnitude_rmse": rms(magnitude_err) if magnitude_err else math.nan,
        "upright_fraction": sum(1 for s in use if s.upright) / len(use),
    }


def write_csv(path: Path, samples: list[SimSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([field.name for field in SimSample.__dataclass_fields__.values()])
        for s in samples:
            writer.writerow(
                [
                    f"{s.t_s:.6f}",
                    f"{s.true_yaw_deg:.6f}",
                    f"{s.true_x_deg:.6f}",
                    f"{s.true_y_deg:.6f}",
                    f"{s.raw_x_deg:.6f}",
                    f"{s.raw_y_deg:.6f}",
                    f"{s.est_x_deg:.6f}",
                    f"{s.est_y_deg:.6f}",
                    f"{s.true_rate_x_rad_s:.9f}",
                    f"{s.true_rate_y_rad_s:.9f}",
                    f"{s.ax_mps2:.6f}",
                    f"{s.ay_mps2:.6f}",
                    f"{s.az_mps2:.6f}",
                    f"{s.gx_rad_s:.9f}",
                    f"{s.gy_rad_s:.9f}",
                    f"{s.gz_rad_s:.9f}",
                    f"{s.kf_bias_x_rad_s:.9f}",
                    f"{s.kf_bias_y_rad_s:.9f}",
                    f"{s.horiz_accel_g:.9f}",
                    int(s.upright),
                ]
            )


def plot(samples: list[SimSample]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for --plot") from exc

    t = [s.t_s for s in samples]
    kf_delta_x = [s.est_x_deg - s.true_x_deg for s in samples]
    kf_delta_y = [s.est_y_deg - s.true_y_deg for s in samples]
    raw_delta_x = [s.raw_x_deg - s.true_x_deg for s in samples]
    raw_delta_y = [s.raw_y_deg - s.true_y_deg for s in samples]
    azimuth_delta = [
        tilt_azimuth_error_deg(s)
        if math.hypot(s.true_x_deg, s.true_y_deg) >= 1.0
        else math.nan
        for s in samples
    ]

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 10))

    axes[0].plot(t, [s.true_x_deg for s in samples], label="truth x", linewidth=2)
    axes[0].plot(t, [s.raw_x_deg for s in samples], label="raw accel x", alpha=0.35)
    axes[0].plot(t, [s.est_x_deg for s in samples], label="KF x")
    axes[0].set_ylabel("tilt x [deg]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, [s.true_y_deg for s in samples], label="truth y", linewidth=2)
    axes[1].plot(t, [s.raw_y_deg for s in samples], label="raw accel y", alpha=0.35)
    axes[1].plot(t, [s.est_y_deg for s in samples], label="KF y")
    axes[1].set_ylabel("tilt y [deg]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, kf_delta_x, label="KF x - truth x")
    axes[2].plot(t, kf_delta_y, label="KF y - truth y")
    axes[2].plot(t, raw_delta_x, label="raw x - truth x", alpha=0.25, linestyle="--")
    axes[2].plot(t, raw_delta_y, label="raw y - truth y", alpha=0.25, linestyle="--")
    axes[2].plot(t, azimuth_delta, label="tilt azimuth error", linestyle=":")
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].set_ylabel("delta [deg]")
    axes[2].set_title("Estimate - truth tilt error")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(t, [s.horiz_accel_g for s in samples], label="horiz accel")
    axes[3].plot(t, [1.0 if s.upright else 0.0 for s in samples], label="upright flag")
    axes[3].axhline(0.3, color="tab:red", linestyle="--", label="upright accel limit")
    axes[3].set_xlabel("time [s]")
    axes[3].set_ylabel("g / flag")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    plt.show()


def parse_vec3(text: str) -> tuple[float, float, float]:
    parts = [float(p.strip()) for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return parts[0], parts[1], parts[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run trajectory or yaw-coupling simulations against tilt_kf.c."
    )
    parser.add_argument("--source-root", type=Path, default=state_estimation_root())
    parser.add_argument(
        "--scenario",
        choices=("trajectory", "yaw-cone"),
        default="trajectory",
        help="motion to simulate; yaw-cone isolates gx coupling at a fixed lean",
    )
    parser.add_argument("--duration", type=float, default=12.0, help="simulation duration [s]")
    parser.add_argument("--rate-hz", type=float, default=200.0, help="IMU sample rate [Hz]")
    parser.add_argument(
        "--yaw-tilt-deg",
        type=float,
        default=20.0,
        help="fixed lean angle for the yaw-cone scenario [deg]",
    )
    parser.add_argument(
        "--yaw-rate-deg-s",
        type=float,
        default=180.0,
        help="body-down (sensor x) rotation rate for the yaw-cone scenario [deg/s]",
    )
    parser.add_argument("--seed", type=int, default=7, help="deterministic noise seed")
    parser.add_argument("--accel-noise-g", type=float, default=0.015)
    parser.add_argument("--gyro-noise-rad-s", type=float, default=0.003)
    parser.add_argument("--accel-bias-g", type=parse_vec3, default=(0.010, -0.006, 0.008))
    parser.add_argument("--gyro-bias-rad-s", type=parse_vec3, default=(0.0, -0.008, 0.011))
    parser.add_argument(
        "--linear-accel-g",
        type=float,
        default=0.12,
        help="mid-simulation non-gravity acceleration disturbance [g]",
    )
    parser.add_argument("--timestamp-jitter-us", type=float, default=0.0)
    parser.add_argument(
        "--csv",
        default=str(Path(tempfile.gettempdir()) / "tilt_kf_sim.csv"),
        help="CSV output path; use --csv '' to disable",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="plot truth/raw/KF traces plus real-minus-expected delta",
    )
    parser.add_argument(
        "--check-rmse-deg",
        type=float,
        default=0.0,
        help="optional nonzero threshold that returns failure if either KF RMSE exceeds it",
    )
    parser.add_argument(
        "--check-azimuth-rmse-deg",
        type=float,
        default=0.0,
        help="optional nonzero failure threshold for tilt-vector alignment RMSE",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    build_dir = Path(tempfile.gettempdir()) / "tilt_kf_sim_build"

    model = ImuModel(
        accel_noise_g=args.accel_noise_g,
        gyro_noise_rad_s=args.gyro_noise_rad_s,
        accel_bias_g=args.accel_bias_g,
        gyro_bias_rad_s=args.gyro_bias_rad_s,
        linear_accel_g=args.linear_accel_g,
        timestamp_jitter_us=args.timestamp_jitter_us,
    )
    csv_path = Path(args.csv) if args.csv else None

    try:
        lib_path = build_c_filter(args.source_root, build_dir)
        api = TiltKfC(lib_path)
        samples = simulate(
            api,
            args.duration,
            args.rate_hz,
            model,
            args.seed,
            args.scenario,
            args.yaw_tilt_deg,
            args.yaw_rate_deg_s,
        )
        stats = metrics(samples, skip_s=0.25)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1

    if csv_path:
        write_csv(csv_path, samples)

    final = samples[-1]
    print("tilt_kf.c simulation complete")
    print(f"  C library: {lib_path}")
    if args.scenario == "yaw-cone":
        print(
            "  scenario: "
            f"yaw cone, tilt={args.yaw_tilt_deg:.1f} deg, "
            f"yaw_rate={args.yaw_rate_deg_s:.1f} deg/s"
        )
    else:
        print("  scenario: two-axis tilt trajectory")
    print(f"  profile: {args.duration:.2f}s, {args.rate_hz:.1f} Hz, {len(samples)} samples")
    print(
        "  sensor model: "
        f"accel_noise={model.accel_noise_g:.3f} g, "
        f"gyro_noise={model.gyro_noise_rad_s:.4f} rad/s, "
        f"linear_disturbance={model.linear_accel_g:.3f} g"
    )
    print(
        "  KF RMSE: "
        f"x={stats['est_x_rmse']:.3f} deg, y={stats['est_y_rmse']:.3f} deg"
    )
    print(
        "  raw accel RMSE: "
        f"x={stats['raw_x_rmse']:.3f} deg, y={stats['raw_y_rmse']:.3f} deg"
    )
    print(
        "  KF max error: "
        f"x={stats['est_x_max']:.3f} deg, y={stats['est_y_max']:.3f} deg"
    )
    if args.scenario == "yaw-cone":
        print(
            "  tilt-vector azimuth error: "
            f"mean={stats['tilt_azimuth_mean']:.3f} deg, "
            f"RMSE={stats['tilt_azimuth_rmse']:.3f} deg, "
            f"max={stats['tilt_azimuth_max']:.3f} deg"
        )
        print(
            "  tilt-magnitude RMSE: "
            f"{stats['tilt_magnitude_rmse']:.3f} deg"
        )
    print(
        "  final: "
        f"truth=({final.true_x_deg:.3f}, {final.true_y_deg:.3f}) deg, "
        f"estimate=({final.est_x_deg:.3f}, {final.est_y_deg:.3f}) deg, "
        f"bias=({final.kf_bias_x_rad_s:.4f}, {final.kf_bias_y_rad_s:.4f}) rad/s"
    )
    print(f"  upright fraction after 0.25s: {100.0 * stats['upright_fraction']:.1f}%")
    if csv_path:
        print(f"  CSV: {csv_path}")

    if args.check_rmse_deg > 0.0:
        worst_rmse = max(stats["est_x_rmse"], stats["est_y_rmse"])
        if worst_rmse > args.check_rmse_deg:
            print(
                f"  RMSE check failed: {worst_rmse:.3f} deg > {args.check_rmse_deg:.3f} deg",
                file=sys.stderr,
            )
            return 1

    if (
        args.check_azimuth_rmse_deg > 0.0
        and stats["tilt_azimuth_rmse"] > args.check_azimuth_rmse_deg
    ):
        print(
            "  azimuth RMSE check failed: "
            f"{stats['tilt_azimuth_rmse']:.3f} deg > "
            f"{args.check_azimuth_rmse_deg:.3f} deg",
            file=sys.stderr,
        )
        return 1

    if args.plot:
        plot(samples)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
