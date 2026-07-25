#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""
Generate simulated sensor data from analytical trajectories for ESKF testing.

Outputs a single unified CSV per scenario with config header, sparse event rows,
and interpolated ground truth at each sensor timestamp.

Usage:
    uv run generate_trajectory.py [--output-dir test-data]
"""

import argparse
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path


# ==========================================================================
# Quaternion utilities (Hamilton convention: w, x, y, z)
# ==========================================================================

def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def quat_from_rotation_vector(rv):
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.array([1.0, 0.5*rv[0], 0.5*rv[1], 0.5*rv[2]])
    ha = 0.5 * angle
    axis = rv / angle
    return np.array([np.cos(ha), *(np.sin(ha) * axis)])


def quat_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [w*w+x*x-y*y-z*z, 2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),     w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z+w*x),     w*w-x*x-y*y+z*z],
    ])


def quat_normalize(q):
    return q / np.linalg.norm(q)


def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))
    theta = np.arccos(np.clip(dot, -1, 1))
    sin_theta = np.sin(theta)
    return (np.sin((1-t)*theta)/sin_theta) * q0 + (np.sin(t*theta)/sin_theta) * q1


# ==========================================================================
# Trajectory Generator
# ==========================================================================

@dataclass
class Segment:
    duration_s: float
    accel_nav: np.ndarray = field(default_factory=lambda: np.zeros(3))
    omega_body: np.ndarray = field(default_factory=lambda: np.zeros(3))


class TrajectoryGenerator:
    GRAVITY = np.array([0.0, 0.0, -9.807])
    GROUND_REACTION = np.array([0.0, 0.0, 9.807])  # cancels gravity on pad

    def __init__(self, segments: list[Segment], dt: float = 1e-4):
        self.segments = segments
        self.dt = dt

    def generate(self):
        total_time = sum(seg.duration_s for seg in self.segments)
        n = int(total_time / self.dt) + 1

        t = np.zeros(n)
        pos = np.zeros((n, 3))
        vel = np.zeros((n, 3))
        quat = np.zeros((n, 4))
        omega = np.zeros((n, 3))

        quat[0] = [1, 0, 0, 0]
        seg_idx = 0
        seg_time = 0.0

        for i in range(1, n):
            while seg_idx < len(self.segments) and \
                  seg_time >= self.segments[seg_idx].duration_s:
                seg_time -= self.segments[seg_idx].duration_s
                seg_idx += 1

            if seg_idx >= len(self.segments):
                a_nav = np.zeros(3)
                w_body = np.zeros(3)
            else:
                seg = self.segments[seg_idx]
                a_nav = seg.accel_nav
                w_body = seg.omega_body

            t[i] = t[i-1] + self.dt

            a_total = a_nav + self.GRAVITY
            vel[i] = vel[i-1] + a_total * self.dt
            pos[i] = pos[i-1] + vel[i-1] * self.dt + 0.5 * a_total * self.dt**2

            rv = w_body * self.dt
            dq = quat_from_rotation_vector(rv)
            quat[i] = quat_normalize(quat_multiply(quat[i-1], dq))
            omega[i] = w_body
            seg_time += self.dt

        t_us = (t * 1e6).astype(np.int64)
        return {
            't_us': t_us, 'pos': pos, 'vel': vel,
            'quat': quat, 'omega': omega, 'dt': self.dt,
        }


# ==========================================================================
# Sensor Simulator
# ==========================================================================

@dataclass
class IMUConfig:
    gyro_noise_std: float = 0.01
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_noise_std: float = 0.02
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rate_hz: float = 800.0


@dataclass
class GPSConfig:
    position_noise_std: float = 2.0
    rate_hz: float = 5.0


@dataclass
class BaroConfig:
    height_noise_std: float = 0.5
    rate_hz: float = 50.0


@dataclass
class MagConfig:
    noise_std: float = 0.05
    rate_hz: float = 100.0


class SensorSimulator:
    GRAV_MPS2 = 9.807

    def __init__(self, truth, rng=None):
        self.truth = truth
        self.rng = rng or np.random.default_rng(42)

    def simulate_imu(self, config: IMUConfig, sensor_id: int = 0):
        truth = self.truth
        dt_truth = truth['dt']
        step = max(1, int(round(1.0 / (config.rate_hz * dt_truth))))

        indices = np.arange(0, len(truth['t_us']), step)
        n = len(indices)

        gyro_samples = np.zeros((n, 4))
        accel_samples = np.zeros((n, 4))

        for j, i in enumerate(indices):
            t = truth['t_us'][i]
            q = truth['quat'][i]
            R = quat_to_rotation_matrix(q)
            R_T = R.T

            omega_b = truth['omega'][i]
            gyro = omega_b + config.gyro_bias + \
                   self.rng.normal(0, config.gyro_noise_std, 3)
            gyro_samples[j] = [t, *gyro]

            if i > 0 and i < len(truth['t_us']) - 1:
                a_total = (truth['vel'][min(i+step, len(truth['vel'])-1)] -
                           truth['vel'][i]) / (step * dt_truth)
            else:
                a_total = np.array([0.0, 0.0, -self.GRAV_MPS2])

            f_nav = a_total - np.array([0.0, 0.0, -self.GRAV_MPS2])
            f_body_g = R_T @ f_nav / self.GRAV_MPS2

            accel = f_body_g + config.accel_bias + \
                    self.rng.normal(0, config.accel_noise_std, 3)
            accel_samples[j] = [t, *accel]

        return {
            'gyro': {'type': 'gyro', 'id': sensor_id, 'data': gyro_samples,
                     'noise_std': config.gyro_noise_std},
            'accel': {'type': 'accel', 'id': sensor_id, 'data': accel_samples,
                      'noise_std': config.accel_noise_std},
        }

    def simulate_gps(self, config: GPSConfig, sensor_id: int = 0):
        truth = self.truth
        dt_truth = truth['dt']
        step = max(1, int(round(1.0 / (config.rate_hz * dt_truth))))

        indices = np.arange(0, len(truth['t_us']), step)
        n = len(indices)
        samples = np.zeros((n, 4))

        for j, i in enumerate(indices):
            pos = truth['pos'][i]
            noise = self.rng.normal(0, config.position_noise_std, 3)
            samples[j] = [truth['t_us'][i], *(pos + noise)]

        return {'type': 'gps', 'id': sensor_id, 'data': samples,
                'noise_std': config.position_noise_std}

    def simulate_baro(self, config: BaroConfig, sensor_id: int = 0):
        truth = self.truth
        dt_truth = truth['dt']
        step = max(1, int(round(1.0 / (config.rate_hz * dt_truth))))

        indices = np.arange(0, len(truth['t_us']), step)
        n = len(indices)
        samples = np.zeros((n, 4))

        for j, i in enumerate(indices):
            h = truth['pos'][i][2]
            noise = self.rng.normal(0, config.height_noise_std)
            samples[j] = [truth['t_us'][i], h + noise, 0, 0]

        return {'type': 'baro', 'id': sensor_id, 'data': samples,
                'noise_std': config.height_noise_std}

    def simulate_mag(self, config: MagConfig, mag_ref=None, sensor_id: int = 0):
        if mag_ref is None:
            mag_ref = np.array([1.0, 0.0, 0.0])

        truth = self.truth
        dt_truth = truth['dt']
        step = max(1, int(round(1.0 / (config.rate_hz * dt_truth))))

        indices = np.arange(0, len(truth['t_us']), step)
        n = len(indices)
        samples = np.zeros((n, 4))

        for j, i in enumerate(indices):
            q = truth['quat'][i]
            R_T = quat_to_rotation_matrix(q).T
            m_body = R_T @ mag_ref
            noise = self.rng.normal(0, config.noise_std, 3)
            m_noisy = m_body + noise
            m_norm = m_noisy / np.linalg.norm(m_noisy)
            samples[j] = [truth['t_us'][i], *m_norm]

        return {'type': 'mag', 'id': sensor_id, 'data': samples,
                'noise_std': config.noise_std}


# ==========================================================================
# Unified CSV Exporter
# ==========================================================================

# Processing priority (must match C-side sensor_priority())
SENSOR_SORT_KEY = {'gyro': 0, 'accel': 1, 'mag': 2, 'gps': 3, 'baro': 4}


def interpolate_truth(truth, t_us):
    """Interpolate ground truth at a specific timestamp via lerp."""
    t_arr = truth['t_us']
    idx = np.searchsorted(t_arr, t_us, side='right') - 1
    idx = np.clip(idx, 0, len(t_arr) - 2)

    t0, t1 = t_arr[idx], t_arr[idx + 1]
    if t1 == t0:
        alpha = 0.0
    else:
        alpha = float(t_us - t0) / float(t1 - t0)

    pos = truth['pos'][idx] + alpha * (truth['pos'][idx+1] - truth['pos'][idx])
    vel = truth['vel'][idx] + alpha * (truth['vel'][idx+1] - truth['vel'][idx])
    quat = quat_slerp(truth['quat'][idx], truth['quat'][idx+1], alpha)

    return quat, pos, vel


class UnifiedExporter:
    """Write unified CSV per scenario with config header + sparse event rows."""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def export(self, scenario: str, truth: dict,
               sensor_channels: list[dict], config: dict):
        """Write single unified CSV.

        config keys: calibration_samples, expected_g, mag_ref, num_imus
        sensor_channels: list of {'type', 'id', 'data', 'noise_std'}
        """
        path = self.output_dir / f"{scenario}.csv"

        # Build event list: (timestamp_us, sort_key, type_str, id, d0, d1, d2)
        events = []
        for ch in sensor_channels:
            sort_key = SENSOR_SORT_KEY.get(ch['type'], 5)
            for row in ch['data']:
                t_us = int(row[0])
                events.append((t_us, sort_key, ch['type'], ch['id'],
                               row[1], row[2], row[3]))

        # Sort by timestamp, then by priority
        events.sort(key=lambda e: (e[0], e[1]))

        with open(path, 'w') as f:
            # Config header
            for key, val in config.items():
                if isinstance(val, (list, np.ndarray)):
                    f.write(f"# {key}={';'.join(f'{v}' for v in val)}\n")
                else:
                    f.write(f"# {key}={val}\n")

            # Sensor noise configs
            for ch in sensor_channels:
                noise = ch.get('noise_std', 0.0)
                if ch['type'] == 'baro':
                    f.write(f"# sensor:{ch['type']}:{ch['id']}:noise={noise}\n")
                else:
                    f.write(f"# sensor:{ch['type']}:{ch['id']}:noise="
                            f"{noise};{noise};{noise}\n")

            # Column header
            f.write("timestamp_us,sensor,id,d0,d1,d2,"
                    "tq_w,tq_x,tq_y,tq_z,"
                    "tp_x,tp_y,tp_z,"
                    "tv_x,tv_y,tv_z\n")

            # Data rows with interpolated truth
            for t_us, _, stype, sid, d0, d1, d2 in events:
                tq, tp, tv = interpolate_truth(truth, t_us)
                f.write(f"{t_us},{stype},{sid},"
                        f"{d0:.8f},{d1:.8f},{d2:.8f},"
                        f"{tq[0]:.8f},{tq[1]:.8f},{tq[2]:.8f},{tq[3]:.8f},"
                        f"{tp[0]:.8f},{tp[1]:.8f},{tp[2]:.8f},"
                        f"{tv[0]:.8f},{tv[1]:.8f},{tv[2]:.8f}\n")

        n_events = len(events)
        print(f"  Wrote {path} ({n_events} events)")


# ==========================================================================
# Test Scenarios
# ==========================================================================

def scenario_static(exporter: UnifiedExporter):
    """Static on pad: calibration + identity quaternion test."""
    print("Generating: static")

    GR = TrajectoryGenerator.GROUND_REACTION
    segments = [Segment(duration_s=2.0, accel_nav=GR)]
    traj = TrajectoryGenerator(segments)
    truth = traj.generate()

    sim = SensorSimulator(truth)
    imu = sim.simulate_imu(IMUConfig(
        gyro_bias=np.array([0.005, -0.003, 0.002]),
        accel_bias=np.array([0.01, -0.02, 0.015]),
    ))

    config = {
        'calibration_samples': 100,
        'expected_g': [0, 0, 1],
        'mag_ref': [1, 0, 0],
        'num_imus': 1,
    }
    mag = sim.simulate_mag(MagConfig())

    channels = [imu['gyro'], imu['accel'], mag]
    exporter.export("static", truth, channels, config)


def scenario_vertical_ascent(exporter: UnifiedExporter):
    """0.5s static + 3s burn + 3s coast with GPS + baro."""
    print("Generating: vertical_ascent")

    GR = TrajectoryGenerator.GROUND_REACTION
    segments = [
        Segment(duration_s=0.5, accel_nav=GR),      # static on pad
        Segment(duration_s=3.0, accel_nav=np.array([0, 0, 30.0])),  # burn (thrust)
        Segment(duration_s=3.0),                      # coast (free fall)
    ]
    traj = TrajectoryGenerator(segments)
    truth = traj.generate()

    sim = SensorSimulator(truth)
    imu = sim.simulate_imu(IMUConfig())
    gps = sim.simulate_gps(GPSConfig())
    baro = sim.simulate_baro(BaroConfig())

    config = {
        'calibration_samples': 50,
        'expected_g': [0, 0, 1],
        'mag_ref': [1, 0, 0],
        'num_imus': 1,
    }
    mag = sim.simulate_mag(MagConfig())

    channels = [imu['gyro'], imu['accel'], gps, baro, mag]
    exporter.export("vertical_ascent", truth, channels, config)


def scenario_pitch_maneuver(exporter: UnifiedExporter):
    """0.5s static + 3s constant pitch rate + 1s hold."""
    print("Generating: pitch_maneuver")

    GR = TrajectoryGenerator.GROUND_REACTION
    segments = [
        Segment(duration_s=0.5, accel_nav=GR),
        Segment(duration_s=3.0, accel_nav=GR, omega_body=np.array([0, 0.5, 0])),
        Segment(duration_s=1.0, accel_nav=GR),
    ]
    traj = TrajectoryGenerator(segments)
    truth = traj.generate()

    sim = SensorSimulator(truth)
    imu = sim.simulate_imu(IMUConfig())

    config = {
        'calibration_samples': 50,
        'expected_g': [0, 0, 1],
        'mag_ref': [1, 0, 0],
        'num_imus': 1,
    }
    mag = sim.simulate_mag(MagConfig())

    channels = [imu['gyro'], imu['accel'], mag]
    exporter.export("pitch_maneuver", truth, channels, config)


def scenario_dual_imu_static(exporter: UnifiedExporter):
    """Static with 2 IMUs having different biases."""
    print("Generating: dual_imu_static")

    GR = TrajectoryGenerator.GROUND_REACTION
    segments = [Segment(duration_s=2.0, accel_nav=GR)]
    traj = TrajectoryGenerator(segments)
    truth = traj.generate()

    sim = SensorSimulator(truth)
    imu0 = sim.simulate_imu(IMUConfig(
        gyro_bias=np.array([0.005, -0.003, 0.002]),
        accel_bias=np.array([0.01, -0.02, 0.015]),
    ), sensor_id=0)
    imu1 = sim.simulate_imu(IMUConfig(
        gyro_bias=np.array([-0.003, 0.004, -0.001]),
        accel_bias=np.array([-0.01, 0.01, -0.005]),
    ), sensor_id=1)

    config = {
        'calibration_samples': 100,
        'expected_g': [0, 0, 1],
        'mag_ref': [1, 0, 0],
        'num_imus': 2,
    }
    mag = sim.simulate_mag(MagConfig())

    channels = [imu0['gyro'], imu0['accel'], imu1['gyro'], imu1['accel'], mag]
    exporter.export("dual_imu_static", truth, channels, config)


def scenario_dual_baro_ascent(exporter: UnifiedExporter):
    """Vertical ascent with 2 baros at different noise levels."""
    print("Generating: dual_baro_ascent")

    GR = TrajectoryGenerator.GROUND_REACTION
    segments = [
        Segment(duration_s=0.5, accel_nav=GR),
        Segment(duration_s=3.0, accel_nav=np.array([0, 0, 30.0])),
        Segment(duration_s=3.0),
    ]
    traj = TrajectoryGenerator(segments)
    truth = traj.generate()

    sim = SensorSimulator(truth)
    imu = sim.simulate_imu(IMUConfig())
    gps = sim.simulate_gps(GPSConfig())
    baro0 = sim.simulate_baro(BaroConfig(height_noise_std=0.5), sensor_id=0)
    baro1 = sim.simulate_baro(BaroConfig(height_noise_std=2.0), sensor_id=1)

    config = {
        'calibration_samples': 50,
        'expected_g': [0, 0, 1],
        'mag_ref': [1, 0, 0],
        'num_imus': 1,
    }
    mag = sim.simulate_mag(MagConfig())

    channels = [imu['gyro'], imu['accel'], gps, baro0, baro1, mag]
    exporter.export("dual_baro_ascent", truth, channels, config)


# ==========================================================================
# Main
# ==========================================================================

def main():
    parser = argparse.ArgumentParser(description="Generate ESKF test data")
    parser.add_argument("--output-dir", default="test-data",
                        help="Output directory for CSV files")
    args = parser.parse_args()

    exporter = UnifiedExporter(args.output_dir)

    scenario_static(exporter)
    scenario_vertical_ascent(exporter)
    scenario_pitch_maneuver(exporter)
    scenario_dual_imu_static(exporter)
    scenario_dual_baro_ascent(exporter)

    print("\nDone. Generated test data in:", args.output_dir)


if __name__ == "__main__":
    main()
