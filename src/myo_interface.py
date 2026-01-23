"""
BioRobotics Lab 1 - Myo Armband Interface
==========================================

This module provides an LSL interface for the Myo Armband.
The Myo streams 8 channels of sEMG at 200 Hz, plus IMU data.

Note: The Myo Armband requires MyoConnect to be running on Windows.
This module reads from the Myo SDK and publishes data to LSL.

Channel Layout (EMG):
- EMG_1 through EMG_8: 8 sEMG channels around the forearm

IMU Data:
- Orientation (quaternion): w, x, y, z
- Accelerometer: x, y, z (in g)
- Gyroscope: x, y, z (in deg/s)

Author: BioRobotics Course
Updated: 2025
"""

import sys
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    import pylsl
    HAS_LSL = True
except ImportError:
    HAS_LSL = False

# Try to import Myo SDK
try:
    # The myo-python package provides the Myo SDK bindings
    import myo
    HAS_MYO = True
except ImportError:
    HAS_MYO = False


@dataclass
class MyoData:
    """Container for Myo sensor data."""
    timestamp: float = 0.0
    emg: list[int] = field(default_factory=lambda: [0] * 8)
    orientation: dict = field(default_factory=lambda: {'w': 0, 'x': 0, 'y': 0, 'z': 0})
    acceleration: dict = field(default_factory=lambda: {'x': 0, 'y': 0, 'z': 0})
    gyroscope: dict = field(default_factory=lambda: {'x': 0, 'y': 0, 'z': 0})
    arm: str = "unknown"
    synced: bool = False
    
    def to_emg_list(self) -> list:
        """Convert to list for LSL streaming."""
        return [
            self.timestamp,
            *self.emg,
            self.orientation['w'],
            self.orientation['x'],
            self.orientation['y'],
            self.orientation['z'],
            self.acceleration['x'],
            self.acceleration['y'],
            self.acceleration['z'],
            self.gyroscope['x'],
            self.gyroscope['y'],
            self.gyroscope['z'],
        ]


if HAS_MYO:
    
    class MyoLSLListener(myo.DeviceListener):
        """
        Myo device listener that publishes data to LSL.
        """
        
        def __init__(self, stream_name: str = "Myo"):
            super().__init__()
            self.stream_name = stream_name
            self.data = MyoData()
            self.sample_count = 0
            self._setup_lsl_outlets()
        
        def _setup_lsl_outlets(self):
            """Create LSL outlets for EMG and IMU data."""
            # EMG outlet (8 channels at 200 Hz)
            emg_info = pylsl.StreamInfo(
                name=f"{self.stream_name}_EMG",
                type='EMG',
                channel_count=8,
                nominal_srate=200,
                channel_format=pylsl.cf_int8,
                source_id=f'{self.stream_name}_EMG'
            )
            
            # Add channel descriptions
            desc = emg_info.desc()
            desc.append_child_value("manufacturer", "Thalmic Labs")
            channels = desc.append_child("channels")
            for i in range(8):
                ch = channels.append_child("channel")
                ch.append_child_value("label", f"EMG_{i+1}")
                ch.append_child_value("unit", "raw")
                ch.append_child_value("type", "EMG")
            
            self.emg_outlet = pylsl.StreamOutlet(emg_info)
            
            # IMU outlet (orientation + accel + gyro = 10 channels at ~50 Hz)
            imu_info = pylsl.StreamInfo(
                name=f"{self.stream_name}_IMU",
                type='IMU',
                channel_count=10,
                nominal_srate=50,
                channel_format=pylsl.cf_float32,
                source_id=f'{self.stream_name}_IMU'
            )
            
            desc = imu_info.desc()
            desc.append_child_value("manufacturer", "Thalmic Labs")
            channels = desc.append_child("channels")
            for label in ['Ori_W', 'Ori_X', 'Ori_Y', 'Ori_Z', 
                         'Acc_X', 'Acc_Y', 'Acc_Z',
                         'Gyro_X', 'Gyro_Y', 'Gyro_Z']:
                ch = channels.append_child("channel")
                ch.append_child_value("label", label)
            
            self.imu_outlet = pylsl.StreamOutlet(imu_info)
            
            print(f"Created LSL outlets: {self.stream_name}_EMG, {self.stream_name}_IMU")
        
        def on_connect(self, device, timestamp, firmware_version):
            """Called when Myo connects."""
            print(f"Myo connected! Firmware: {firmware_version}")
            device.set_stream_emg(myo.StreamEmg.enabled)
            device.vibrate(myo.VibrationType.short)
        
        def on_disconnect(self, device, timestamp):
            """Called when Myo disconnects."""
            print("Myo disconnected")
        
        def on_arm_sync(self, device, timestamp, arm, x_direction, rotation, warmup_state):
            """Called when Myo syncs to an arm."""
            self.data.arm = arm.name
            self.data.synced = True
            print(f"Synced to {arm.name} arm")
            device.vibrate(myo.VibrationType.medium)
        
        def on_arm_unsync(self, device, timestamp):
            """Called when Myo unsyncs."""
            self.data.synced = False
            print("Myo unsynced")
        
        def on_emg(self, device, timestamp, emg):
            """Called when EMG data is received (200 Hz)."""
            self.data.emg = list(emg)
            self.data.timestamp = timestamp
            
            # Push to LSL
            self.emg_outlet.push_sample(self.data.emg)
            self.sample_count += 1
        
        def on_orientation(self, device, timestamp, orientation):
            """Called when orientation data is received (~50 Hz)."""
            self.data.orientation = {
                'w': orientation.w,
                'x': orientation.x,
                'y': orientation.y,
                'z': orientation.z,
            }
        
        def on_accelerometor(self, device, timestamp, acceleration):
            """Called when accelerometer data is received."""
            self.data.acceleration = {
                'x': acceleration.x,
                'y': acceleration.y,
                'z': acceleration.z,
            }
        
        def on_gyroscope(self, device, timestamp, gyroscope):
            """Called when gyroscope data is received."""
            self.data.gyroscope = {
                'x': gyroscope.x,
                'y': gyroscope.y,
                'z': gyroscope.z,
            }
            
            # Push IMU data (after we have all components)
            imu_sample = [
                self.data.orientation['w'],
                self.data.orientation['x'],
                self.data.orientation['y'],
                self.data.orientation['z'],
                self.data.acceleration['x'],
                self.data.acceleration['y'],
                self.data.acceleration['z'],
                self.data.gyroscope['x'],
                self.data.gyroscope['y'],
                self.data.gyroscope['z'],
            ]
            self.imu_outlet.push_sample(imu_sample)


class MyoStreamer:
    """
    High-level interface for streaming Myo data to LSL.
    
    Example
    -------
    >>> streamer = MyoStreamer()
    >>> streamer.start()
    >>> # ... do something ...
    >>> streamer.stop()
    """
    
    def __init__(self, stream_name: str = "Myo"):
        """
        Initialize the Myo streamer.
        
        Parameters
        ----------
        stream_name : str
            Base name for the LSL streams
        """
        if not HAS_MYO:
            raise ImportError(
                "myo-python not available. Install with: pip install myo-python\n"
                "Also ensure MyoConnect is installed and running."
            )
        
        if not HAS_LSL:
            raise ImportError("pylsl not available. Install with: pip install pylsl")
        
        self.stream_name = stream_name
        self.hub = None
        self.listener = None
        self._running = False
        self._thread = None
    
    def start(self):
        """Start streaming Myo data to LSL."""
        if self._running:
            print("Already running!")
            return
        
        print("Initializing Myo SDK...")
        myo.init()
        
        self.hub = myo.Hub()
        self.listener = MyoLSLListener(self.stream_name)
        
        # Set locking policy to none (always unlocked)
        self.hub.set_locking_policy(myo.LockingPolicy.none)
        
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        
        print("Myo streamer started. Waiting for device...")
        print("Make sure MyoConnect is running and the armband is on.")
    
    def _run_loop(self):
        """Main run loop."""
        while self._running:
            self.hub.run(self.listener.on_event, 100)
    
    def stop(self):
        """Stop streaming."""
        self._running = False
        
        if self._thread:
            self._thread.join(timeout=2.0)
        
        if self.hub:
            self.hub.shutdown()
        
        if self.listener:
            print(f"Streamed {self.listener.sample_count} EMG samples")
        
        print("Myo streamer stopped")
    
    @property
    def is_synced(self) -> bool:
        """Check if Myo is synced to an arm."""
        return self.listener and self.listener.data.synced
    
    @property
    def sample_count(self) -> int:
        """Get the number of samples streamed."""
        return self.listener.sample_count if self.listener else 0


# Fallback for when Myo SDK is not available
class MockMyoStreamer:
    """
    Mock Myo streamer for testing without hardware.
    
    Generates synthetic EMG-like data for testing the pipeline.
    """
    
    def __init__(self, stream_name: str = "MockMyo"):
        if not HAS_LSL:
            raise ImportError("pylsl not available")
        
        self.stream_name = stream_name
        self._running = False
        self._thread = None
        self.sample_count = 0
        
        # Create LSL outlet
        info = pylsl.StreamInfo(
            name=f"{stream_name}_EMG",
            type='EMG',
            channel_count=8,
            nominal_srate=200,
            channel_format=pylsl.cf_float32,
            source_id=f'{stream_name}_EMG'
        )
        
        desc = info.desc()
        channels = desc.append_child("channels")
        for i in range(8):
            ch = channels.append_child("channel")
            ch.append_child_value("label", f"EMG_{i+1}")
        
        self.outlet = pylsl.StreamOutlet(info)
    
    def start(self):
        """Start generating mock data."""
        self._running = True
        self._thread = threading.Thread(target=self._generate_data, daemon=True)
        self._thread.start()
        print(f"Mock Myo started: {self.stream_name}_EMG")
    
    def _generate_data(self):
        """Generate synthetic EMG data."""
        t = 0
        dt = 1.0 / 200  # 200 Hz
        
        while self._running:
            # Generate 8 channels of synthetic EMG
            # Mix of noise + some sinusoidal components
            sample = []
            for ch in range(8):
                noise = np.random.randn() * 10
                signal = 20 * np.sin(2 * np.pi * (10 + ch) * t)
                # Add occasional bursts (muscle activation)
                if np.random.random() < 0.01:
                    signal += 50 * np.random.randn()
                sample.append(noise + signal)
            
            self.outlet.push_sample(sample)
            self.sample_count += 1
            t += dt
            
            time.sleep(dt * 0.95)  # Slight speedup to prevent drift
    
    def stop(self):
        """Stop generating data."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        print(f"Mock Myo stopped. Generated {self.sample_count} samples.")
    
    @property
    def is_synced(self) -> bool:
        return self._running


def main():
    """Command-line interface for Myo streaming."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Myo Armband LSL Streamer")
    parser.add_argument("--name", default="Myo", help="Stream name")
    parser.add_argument("--mock", action="store_true", help="Use mock data (no hardware)")
    parser.add_argument("--duration", type=float, default=0, help="Duration in seconds (0 = indefinite)")
    args = parser.parse_args()
    
    if args.mock:
        streamer = MockMyoStreamer(args.name)
    else:
        if not HAS_MYO:
            print("Myo SDK not available. Use --mock for testing.")
            print("To install: pip install myo-python")
            return 1
        streamer = MyoStreamer(args.name)
    
    try:
        streamer.start()
        
        if args.duration > 0:
            print(f"Running for {args.duration} seconds...")
            time.sleep(args.duration)
        else:
            print("Press Ctrl+C to stop...")
            while True:
                time.sleep(1)
                print(f"Samples: {streamer.sample_count}", end='\r')
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        streamer.stop()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
