#!/usr/bin/env python3
"""
EKF C Code Test Setup with Mock Headers and Build System

This script creates mock headers for missing dependencies and builds
the EKF library for testing.
"""

import ctypes
import numpy as np
import os
import subprocess
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import math
import random

def quat_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi # Radians

def get_expected_accel(roll_deg, pitch_deg, yaw_deg=0):
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    
    gx = -np.sin(p)
    gy = np.sin(r) * np.cos(p)
    gz = np.cos(r) * np.cos(p)

    sg = [0.02 * random.random() * random.choice([-1,1]), 0.02 * random.random() * random.choice([-1,1]), 0.02 * random.random() * random.choice([-1,1])]

    return np.array([gx+sg[0], gy+sg[1], gz+sg[2]])

class EKFTestSetup:
    """Handles building and setting up the EKF test environment"""
    
    def __init__(self, source_dir=None):
        if source_dir is None:
            # Default: source files are in ../src/ and ../include/ relative to this script
            self.source_dir = Path(__file__).resolve().parent.parent / "src"
            self.include_dir = Path(__file__).resolve().parent.parent / "include"
        else:
            self.source_dir = Path(source_dir)
            self.include_dir = self.source_dir
        self.test_dir = Path("ekf_test_build")
        self.lib_name = "ekf.so" if sys.platform != "win32" else "ekf.dll"
    
    def check_required_files(self):
        """Check if required source files exist"""
        required_files = ['ekf.c', 'ekf.h']
        optional_files = ['quaternion.c', 'body.c', 'matrix.c', 'matrix.h', 
                         'quaternion.h', 'body.h', 'state.c', 'state.h']
        
        missing_required = []
        available_optional = []
        
        for f in required_files:
            if not (self.source_dir / f).exists():
                missing_required.append(f)
        
        for f in optional_files:
            if (self.source_dir / f).exists():
                available_optional.append(f)
        
        if missing_required:
            print(f"ERROR: Missing required files: {missing_required}")
            return False
        
        print(f"Found source files: {required_files + available_optional}")
        return True, available_optional
    
    def build_library(self, source_files):
        """Compile the EKF library"""
        
        print("\nBuilding EKF library...")
        
        self.test_dir.mkdir(exist_ok=True)
        
        # Prepare compilation command - compile directly from source directory
        c_files = [f for f in source_files if f.endswith('.c')]
        
        compile_cmd = [
            'gcc',
            '-shared',
            '-fPIC',
            '-I' + str(self.include_dir),  # Header files
            '-o', str(self.test_dir / self.lib_name),
            *[str(self.source_dir / f) for f in c_files],
            '-lm',
            '-std=c99'
        ]
        
        print(f"Compile command: {' '.join(compile_cmd)}")
        
        try:
            result = subprocess.run(compile_cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print("COMPILATION FAILED:")
                print(result.stderr)
                return False
            print(f"✓ Successfully built {self.lib_name}")
            return True
        except FileNotFoundError:
            print("ERROR: gcc compiler not found. Please install gcc.")
            return False
        except Exception as e:
            print(f"ERROR during compilation: {e}")
            return False
    
    def setup(self):
        """Run complete setup process"""
        result = self.check_required_files()
        if not result:
            return False
        
        success, source_files = result
        all_files = ['ekf.c', 'ekf.h'] + source_files
        
        if not self.build_library(all_files):
            return False
        
        return True
    
    def get_library_path(self):
        """Get path to compiled library"""
        return str(self.test_dir / self.lib_name)


class EKFTester:
    """Python wrapper for testing the EKF C implementation"""
    
    def __init__(self, lib_path):
        """Initialize the EKF tester by loading the shared library"""
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"Library not found at {lib_path}")
        
        self.lib = ctypes.CDLL(lib_path)
        self._setup_function_signatures()
    
    def _setup_function_signatures(self):
        """Define C function signatures for ctypes"""
        
        # init_ekf_orientation
        self.lib.init_ekf_orientation.argtypes = [
            ctypes.POINTER(ctypes.c_float * 4),  # process_noise[4][4]
            ctypes.POINTER(ctypes.c_float * 3),  # measurement_noise[3][3]
            ctypes.POINTER(ctypes.c_float)        # expected_g[3]
        ]
        self.lib.init_ekf_orientation.restype = None
        
        # init_ekf_body
        self.lib.init_ekf_body.argtypes = [
            ctypes.POINTER(ctypes.c_float * 6),  # process_noise[6][6]
            ctypes.POINTER(ctypes.c_float * 3)   # measurement_noise[3][3]
        ]
        self.lib.init_ekf_body.restype = None
        
        # tick_ekf_orientation
        self.lib.tick_ekf_orientation.argtypes = [
            ctypes.c_float,                      # deltaTime
            ctypes.POINTER(ctypes.c_float),      # gyro[3]
            ctypes.POINTER(ctypes.c_float)       # accel[3]
        ]
        self.lib.tick_ekf_orientation.restype = None
        
        # tick_ekf_body
        self.lib.tick_ekf_body.argtypes = [
            ctypes.c_float,                      # deltaTime
            ctypes.POINTER(ctypes.c_float),      # accel[3]
            ctypes.POINTER(ctypes.c_float)       # gps_pos[3]
        ]
        self.lib.tick_ekf_body.restype = None
        
        # get_state
        self.lib.get_state.argtypes = [
            ctypes.POINTER(ctypes.c_float),      # quaternion[4]
            ctypes.POINTER(ctypes.c_float),      # position[3]
            ctypes.POINTER(ctypes.c_float)       # velocity[3]
        ]
        self.lib.get_state.restype = None
    
    def init_orientation(self, process_noise, measurement_noise, expected_g):
        """Initialize orientation EKF"""
        process_noise = np.array(process_noise, dtype=np.float32)
        measurement_noise = np.array(measurement_noise, dtype=np.float32)
        expected_g = np.array(expected_g, dtype=np.float32)
        
        process_c = ((ctypes.c_float * 4) * 4)()
        for i in range(4):
            for j in range(4):
                process_c[i][j] = process_noise[i][j]
        
        measure_c = ((ctypes.c_float * 3) * 3)()
        for i in range(3):
            for j in range(3):
                measure_c[i][j] = measurement_noise[i][j]
        
        gravity_c = expected_g.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.init_ekf_orientation(process_c, measure_c, gravity_c)
    
    def init_body(self, process_noise, measurement_noise):
        """Initialize body state EKF"""
        process_noise = np.array(process_noise, dtype=np.float32)
        measurement_noise = np.array(measurement_noise, dtype=np.float32)
        
        process_c = ((ctypes.c_float * 6) * 6)()
        for i in range(6):
            for j in range(6):
                process_c[i][j] = process_noise[i][j]
        
        measure_c = ((ctypes.c_float * 3) * 3)()
        for i in range(3):
            for j in range(3):
                measure_c[i][j] = measurement_noise[i][j]
        
        self.lib.init_ekf_body(process_c, measure_c)
    
    def tick_orientation(self, dt, gyro, accel):
        """Update orientation EKF"""
        gyro = np.array(gyro, dtype=np.float32)
        accel = np.array(accel, dtype=np.float32)
        
        gyro_c = gyro.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        accel_c = accel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.tick_ekf_orientation(ctypes.c_float(dt), gyro_c, accel_c)
    
    def tick_body(self, dt, accel, gps_pos):
        """Update body state EKF"""
        accel = np.array(accel, dtype=np.float32)
        gps_pos = np.array(gps_pos, dtype=np.float32)
        
        accel_c = accel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        gps_c = gps_pos.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.tick_ekf_body(ctypes.c_float(dt), accel_c, gps_c)
    
    def get_state(self):
        """Get current EKF state"""
        quat = np.zeros(4, dtype=np.float32)
        pos = np.zeros(3, dtype=np.float32)
        vel = np.zeros(3, dtype=np.float32)
        
        quat_c = quat.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        pos_c = pos.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        vel_c = vel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.get_state(quat_c, pos_c, vel_c)
        
        return quat, pos, vel

class Entry:
    accel = [0, 0, 0]
    gyro = [0, 0, 0]
    gps = [0, 0, 0]
    dt = 0

    def __init__(self, accel, gyro, gps, dt):
        self.accel = accel[:]
        self.gyro = gyro[:]
        self.gps = gps[:]
        self.dt = dt
 
class Truth:
    pos = [0, 0, 0]
    vel = [0, 0, 0]
    quat = [0, 0, 0, 0]

    def __init__(self, pos, vel, quat):
        self.pos = pos[:]
        self.vel = vel[:]
        self.quat = quat[:]

class Data:
    entries = []
    expected_values = []
    entryCount = 0
    index = 0

    def __init__(self):
        entries = []
        entryCount = 0

    def add_entry(self, entry, expected_value):
        self.entries.append(entry)
        self.expected_values.append(expected_value)

    def get_next_data(self):
        if (self.index >= self.entryCount):
            return -1
        
        index = self.index
        self.index += 1

        return (self.entries[index], self.expected_values[index])

class Tester:
    process_noise = np.eye(4, dtype=np.float32) * 0.001
    measurement_noise = np.eye(3, dtype=np.float32) * 0.1
    expected_g = np.array([0.0, 0.0, 1], dtype=np.float32)

    def __init__(self, ekf, data):
        ekf.init_orientation(self.process_noise, self.measurement_noise, self.expected_g)
        ekf.init_body(np.eye(6) * 0.01, np.eye(3) * 0.1)
        self.data = data

    def run_tests(self, ekf):
        for i in range(self.data.entryCount):
            input_data = self.data.entries[i]
            true_data = self.data.expected_values[i]

            ekf.tick_orientation(input_data.dt, input_data.gyro, input_data.accel)
            ekf.tick_body(input_data.dt, input_data.accel, input_data.gps)





# ============================================
# TEST CASES
# ============================================

def test_orientation_static(ekf):
    """Test orientation EKF with static readings"""
    print("\n" + "="*60)
    print("TEST 1: Orientation EKF - Static (No Rotation)")
    print("="*60)
    
    process_noise = np.eye(4, dtype=np.float32) * 0.001
    measurement_noise = np.eye(3, dtype=np.float32) * 0.1
    expected_g = np.array([0.0, 0.0, 1], dtype=np.float32)
    
    ekf.init_orientation(process_noise, measurement_noise, expected_g)
    ekf.init_body(np.eye(6) * 0.01, np.eye(3) * 0.1)
    
    dt = 0.01
    gyro = np.array([0.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 1])
    
    print(f"Simulating {100} steps with dt={dt}s")
    print(f"Gyro: {gyro}, Accel: {accel}\n")
    
    for i in range(100):
        ekf.tick_orientation(dt, gyro, accel)
        
        if i % 25 == 0:
            quat, _, _ = ekf.get_state()
            norm = np.linalg.norm(quat)
            print(f"Step {i:3d}: quat=[{quat[0]:7.4f}, {quat[1]:7.4f}, "
                  f"{quat[2]:7.4f}, {quat[3]:7.4f}], norm={norm:.6f}")
    
    quat, _, _ = ekf.get_state()
    print(f"\n✓ Final quaternion norm: {np.linalg.norm(quat):.6f} (should be ~1.0)")


def test_body_motion(ekf):
    """Test body state EKF with constant acceleration"""
    print("\n" + "="*60)
    print("TEST 2: Body State EKF - Constant Acceleration")
    print("="*60)
    
    process_noise = np.eye(6, dtype=np.float32) * 0.01
    measurement_noise = np.eye(3, dtype=np.float32) * 1.0

    delta = []
    
    ekf.init_orientation(np.eye(4) * 0.001, np.eye(3) * 0.1, np.array([0, 0, -1]))
    ekf.init_body(process_noise, measurement_noise)
    
    dt = 0.01
    accel = np.array([1.0, 0.0, 0.0])
    
    print(f"Simulating motion with acceleration: {accel} m/s²")
    print(f"Time step: {dt}s, Total steps: 200\n")
    
    for i in range(200):
        true_pos = np.array([0.5 * accel[0] * (i * dt)**2, 0.0, 0.0])
        noise = np.random.randn(3) * 0.1
        gps_pos = true_pos + noise
        
        ekf.tick_body(dt, accel, gps_pos)
        _, pos, vel = ekf.get_state()

        ds = [0, 0, 0]
        for j in range(3):
            ds[j] = float(true_pos[j] - pos[j])

        delta.append(ds)
        
        if i % 50 == 0:
           
            print(f"Step {i:3d}: pos=[{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}], "
                  f"vel=[{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}]")
            
    _, final_pos, final_vel = ekf.get_state()
    expected_vel = accel * (200 * dt)
    print(f"\nFinal velocity:   [{final_vel[0]:6.3f}, {final_vel[1]:6.3f}, {final_vel[2]:6.3f}]")
    print(f"Expected velocity: [{expected_vel[0]:6.3f}, {expected_vel[1]:6.3f}, {expected_vel[2]:6.3f}]")
    print(f"\nFinal pos:   [{final_pos[0]:6.3f}, {final_pos[1]:6.3f}, {final_pos[2]:6.3f}]")
    print(f"Expected pos: [{true_pos[0]:6.3f}, {true_pos[1]:6.3f}, {true_pos[2]:6.3f}]")

    delta_array = np.array(delta)
    time_steps = np.arange(len(delta)) * dt

    # plot(delta_array, time_steps)
    


def test_rotation(ekf):
    """Test orientation EKF with rotation"""
    print("\n" + "="*60)
    print("TEST 3: Orientation EKF - Z-Axis Rotation")
    print("="*60)
    
    process_noise = np.eye(4, dtype=np.float32) * 0.01
    measurement_noise = np.eye(3, dtype=np.float32) * 0.1
    expected_g = np.array([0.0, 0.0, 1], dtype=np.float32)
    delta = []
    
    ekf.init_orientation(process_noise, measurement_noise, expected_g)
    ekf.init_body(np.eye(6) * 0.01, np.eye(3) * 0.1)
    
    dt = 0.01
    def omega_z(t):
        return min(t / 400, 0.04)  # rad/s
    
    expected_angle = 0
    
    # print(f"Rotating around Z-axis at {omega_z} rad/s ({omega_z*180/np.pi:.1f}°/s)")
    print(f"Time step: {dt}s, Total steps: 200\n")
    
    for i in range(1000):
        gyro_noise = random.choice([-1,1]) * random.random() * 0.01 * math.pi / 180
        gyro = np.array([0.0, omega_z(i) + gyro_noise, 0.0])
        accel = np.array(get_expected_accel(0, expected_angle, 0))
        
        ekf.tick_orientation(dt, gyro, accel)
        quat, _, _ = ekf.get_state()

        euler = quat_to_euler(quat[0], quat[1], quat[2], quat[3])
        delta.append([0-euler[0], expected_angle-euler[1],0-euler[2]])

        expected_angle += omega_z(i) * dt * 180/np.pi

        if i % 100 == 0:
            print(euler)
            angle = 2 * np.arccos(np.clip(quat[0], -1, 1)) * 180/np.pi
            print(f"Step {i:3d}: quat=[{quat[0]:7.4f}, {quat[1]:7.4f}, "
                  f"{quat[2]:7.4f}, {quat[3]:7.4f}], angle={angle:6.2f}°")
    
    delta_array = np.array(delta)
    time_steps = np.arange(len(delta)) * dt

    quat, _, _ = ekf.get_state()
    total_angle = 2 * np.arccos(np.clip(quat[0], -1, 1)) * 180/np.pi
    print(f"\nTotal rotation: {total_angle:.2f}° (expected ~{expected_angle:.2f}°)")

    plot(delta_array, time_steps)


def main():
    print("="*60)
    print("EKF C Code Test Suite with Auto-Build")
    print("="*60)
    
    # Setup and build
    setup = EKFTestSetup()
    
    if not setup.setup():
        print("\nERRRR.")
        return 1
    
    print("\n✓ Build successful!")
    
    # Run tests
    try:
        ekf = EKFTester(setup.get_library_path())
        
        test_orientation_static(ekf)
        test_body_motion(ekf)
        test_rotation(ekf)
        
        print("\n" + "="*60)
        print("chilling")
        print("="*60)
        
        return 0
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1

def plot(delta_array, time_steps):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Position Error (True - Estimated) Over Time', fontsize=14, fontweight='bold')
    
    # X-axis error
    axes[0, 0].plot(time_steps, delta_array[:, 0], 'r-', linewidth=1.5)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].set_title('X-axis Error')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Y-axis error
    axes[0, 1].plot(time_steps, delta_array[:, 1], 'g-', linewidth=1.5)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Error (m)')
    axes[0, 1].set_title('Y-axis Error')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Z-axis error
    axes[1, 0].plot(time_steps, delta_array[:, 2], 'b-', linewidth=1.5)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Error (m)')
    axes[1, 0].set_title('Z-axis Error')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # All axes combined
    axes[1, 1].plot(time_steps, delta_array[:, 0], 'r-', linewidth=1.5, label='X-axis', alpha=0.8)
    axes[1, 1].plot(time_steps, delta_array[:, 1], 'g-', linewidth=1.5, label='Y-axis', alpha=0.8)
    axes[1, 1].plot(time_steps, delta_array[:, 2], 'b-', linewidth=1.5, label='Z-axis', alpha=0.8)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Error (m)')
    axes[1, 1].set_title('All Axes Combined')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    axes[1, 1].legend(loc='best')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    sys.exit(main())