"""
BioRobotics Lab 1 - Myo Armband Interface
==========================================

This module provides an LSL interface for the Myo Armband using pyomyo.
pyomyo communicates directly via Bluetooth - NO SDK required!

The Myo streams 8 channels of sEMG at 200 Hz, plus IMU data.

Channel Layout (EMG):
- EMG_1 through EMG_8: 8 sEMG channels around the forearm

Requirements:
- pyomyo: pip install pyomyo
- Myo armband paired via Windows Bluetooth settings

Author: BioRobotics Course
Updated: 2025
"""

import sys
import os
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, List
from datetime import datetime
from pathlib import Path
from collections import deque

import numpy as np

try:
    import pylsl

    HAS_LSL = True
except ImportError:
    HAS_LSL = False
    print("Warning: pylsl not available. Install with: pip install pylsl")

# Try to import pyomyo (preferred - no SDK needed)
HAS_PYOMYO = False
try:
    from pyomyo import Myo, emg_mode

    HAS_PYOMYO = True
    print("pyomyo available - will use direct Bluetooth connection (no SDK needed)")
except ImportError:
    print("pyomyo not available. Install with: pip install pyomyo")

# Try to import myo-python as fallback
HAS_MYO_PYTHON = False
if not HAS_PYOMYO:
    try:
        import myo

        HAS_MYO_PYTHON = True
        print("myo-python available as fallback")
    except ImportError:
        pass


@dataclass
class MyoData:
    """Container for Myo data."""
    emg: List[int] = field(default_factory=lambda: [0] * 8)
    orientation: dict = field(default_factory=lambda: {'w': 0, 'x': 0, 'y': 0, 'z': 0})
    acceleration: dict = field(default_factory=lambda: {'x': 0, 'y': 0, 'z': 0})
    gyroscope: dict = field(default_factory=lambda: {'x': 0, 'y': 0, 'z': 0})
    timestamp: float = 0.0
    arm: str = "unknown"
    synced: bool = False


class MyoLSLStreamer:
    """
    Streams Myo Armband data to LSL using pyomyo.

    pyomyo communicates directly via Bluetooth - no SDK required!

    Example
    -------
    >>> streamer = MyoLSLStreamer()
    >>> streamer.start()
    >>> # ... collect data ...
    >>> streamer.stop()
    """

    def __init__(self, stream_name: str = "Myo", emg_mode_setting: str = "raw"):
        """
        Initialize the Myo LSL streamer.

        Parameters
        ----------
        stream_name : str
            Base name for the LSL streams
        emg_mode_setting : str
            EMG mode: "raw" (200Hz, -128 to 127), "filtered" (200Hz, filtered),
            or "preprocessed" (50Hz, rectified)
        """
        if not HAS_PYOMYO:
            raise ImportError(
                "pyomyo not available.\n"
                "Install with: pip install pyomyo\n\n"
                "pyomyo works without the Myo SDK - it communicates directly via Bluetooth!"
            )

        if not HAS_LSL:
            raise ImportError("pylsl not available. Install with: pip install pylsl")

        self.stream_name = stream_name
        self.emg_mode_setting = emg_mode_setting

        # Set EMG mode
        if emg_mode_setting == "raw":
            self.emg_mode = emg_mode.RAW
            self.emg_rate = 200
        elif emg_mode_setting == "filtered":
            self.emg_mode = emg_mode.FILTERED
            self.emg_rate = 200
        else:  # preprocessed
            self.emg_mode = emg_mode.PREPROCESSED
            self.emg_rate = 50

        self.myo = None
        self.data = MyoData()
        self.sample_count = 0
        self._running = False
        self._thread = None

        # LSL outlets
        self.emg_outlet = None
        self.imu_outlet = None

    def _setup_lsl_outlets(self):
        """Create LSL outlets for EMG and IMU data."""
        # EMG outlet (8 channels)
        emg_info = pylsl.StreamInfo(
            name=f"{self.stream_name}_EMG",
            type='EMG',
            channel_count=8,
            nominal_srate=self.emg_rate,
            channel_format=pylsl.cf_int8 if self.emg_mode == emg_mode.RAW else pylsl.cf_float32,
            source_id=f'{self.stream_name}_EMG'
        )

        # Add channel descriptions
        desc = emg_info.desc()
        desc.append_child_value("manufacturer", "Thalmic Labs")
        desc.append_child_value("emg_mode", self.emg_mode_setting)
        channels = desc.append_child("channels")
        for i in range(8):
            ch = channels.append_child("channel")
            ch.append_child_value("label", f"EMG_{i + 1}")
            ch.append_child_value("unit", "raw" if self.emg_mode == emg_mode.RAW else "uV")
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

        print(f"Created LSL outlets: {self.stream_name}_EMG ({self.emg_rate}Hz), {self.stream_name}_IMU (50Hz)")

    def _emg_callback(self, emg, movement):
        """Called when EMG data is received."""
        self.data.emg = list(emg)
        self.data.timestamp = time.time()

        # Push to LSL
        if self.emg_outlet:
            self.emg_outlet.push_sample(self.data.emg)
        self.sample_count += 1

    def _imu_callback(self, quat, acc, gyro):
        """Called when IMU data is received."""
        self.data.orientation = {'w': quat[0], 'x': quat[1], 'y': quat[2], 'z': quat[3]}
        self.data.acceleration = {'x': acc[0], 'y': acc[1], 'z': acc[2]}
        self.data.gyroscope = {'x': gyro[0], 'y': gyro[1], 'z': gyro[2]}

        # Push to LSL
        if self.imu_outlet:
            imu_sample = [
                quat[0], quat[1], quat[2], quat[3],
                acc[0], acc[1], acc[2],
                gyro[0], gyro[1], gyro[2],
            ]
            self.imu_outlet.push_sample(imu_sample)

    def start(self):
        """Start streaming Myo data to LSL."""
        if self._running:
            print("Already running!")
            return

        print("Connecting to Myo armband via Bluetooth...")
        print("Make sure the Myo is:")
        print("  1. Charged and powered on")
        print("  2. Paired in Windows Bluetooth settings")
        print("  3. Not connected to MyoConnect (close it if running)")

        # Setup LSL outlets
        self._setup_lsl_outlets()

        # Create Myo object
        self.myo = Myo(mode=self.emg_mode)
        self.myo.connect()

        # Set callbacks
        self.myo.add_emg_handler(self._emg_callback)
        self.myo.add_imu_handler(self._imu_callback)

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        print("Myo streamer started!")
        print(f"Streaming EMG at {self.emg_rate}Hz, IMU at 50Hz")

    def _run_loop(self):
        """Main run loop."""
        try:
            while self._running:
                self.myo.run()
        except Exception as e:
            print(f"Error in Myo run loop: {e}")
            self._running = False

    def stop(self):
        """Stop streaming."""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)

        if self.myo:
            try:
                self.myo.disconnect()
            except:
                pass

        print(f"Streamed {self.sample_count} EMG samples")
        print("Myo streamer stopped")

    @property
    def is_connected(self) -> bool:
        """Check if Myo is connected."""
        return self._running and self.myo is not None

    def vibrate(self, duration: str = "short"):
        """
        Vibrate the Myo.

        Parameters
        ----------
        duration : str
            "short", "medium", or "long"
        """
        if self.myo:
            if duration == "short":
                self.myo.vibrate(1)
            elif duration == "medium":
                self.myo.vibrate(2)
            else:
                self.myo.vibrate(3)


# Alias for backwards compatibility
MyoStreamer = MyoLSLStreamer


class MockMyoStreamer:
    """
    Mock Myo streamer for testing without hardware.

    Generates synthetic EMG data that looks realistic.
    """

    def __init__(self, stream_name: str = "MockMyo"):
        if not HAS_LSL:
            raise ImportError("pylsl not available. Install with: pip install pylsl")

        self.stream_name = stream_name
        self.sample_count = 0
        self._running = False
        self._thread = None
        self.emg_outlet = None
        self.imu_outlet = None

    def _setup_lsl_outlets(self):
        """Create LSL outlets."""
        # EMG outlet
        emg_info = pylsl.StreamInfo(
            name=f"{self.stream_name}_EMG",
            type='EMG',
            channel_count=8,
            nominal_srate=200,
            channel_format=pylsl.cf_int8,
            source_id=f'{self.stream_name}_EMG'
        )

        desc = emg_info.desc()
        desc.append_child_value("manufacturer", "Mock")
        channels = desc.append_child("channels")
        for i in range(8):
            ch = channels.append_child("channel")
            ch.append_child_value("label", f"EMG_{i + 1}")

        self.emg_outlet = pylsl.StreamOutlet(emg_info)

        # IMU outlet
        imu_info = pylsl.StreamInfo(
            name=f"{self.stream_name}_IMU",
            type='IMU',
            channel_count=10,
            nominal_srate=50,
            channel_format=pylsl.cf_float32,
            source_id=f'{self.stream_name}_IMU'
        )
        self.imu_outlet = pylsl.StreamOutlet(imu_info)

        print(f"Created mock LSL outlets: {self.stream_name}_EMG, {self.stream_name}_IMU")

    def start(self):
        """Start generating mock data."""
        if self._running:
            return

        self._setup_lsl_outlets()
        self._running = True
        self._thread = threading.Thread(target=self._generate_data, daemon=True)
        self._thread.start()
        print("Mock Myo streamer started")

    def _generate_data(self):
        """Generate synthetic EMG and IMU data."""
        t = 0
        while self._running:
            # Generate 8 channels of synthetic EMG
            # Base noise + occasional bursts
            emg = []
            for ch in range(8):
                # Base noise
                val = np.random.normal(0, 5)

                # Add periodic bursts (simulating muscle activation)
                if np.sin(2 * np.pi * 0.5 * t + ch * 0.5) > 0.7:
                    val += np.random.normal(40, 15)

                # Add 60Hz interference
                val += 2 * np.sin(2 * np.pi * 60 * t)

                # Clamp to int8 range
                val = int(np.clip(val, -128, 127))
                emg.append(val)

            self.emg_outlet.push_sample(emg)
            self.sample_count += 1

            # Generate IMU data at lower rate
            if self.sample_count % 4 == 0:  # 50Hz
                imu = [
                    1.0, 0.0, 0.0, 0.0,  # Quaternion (identity)
                    0.0, 0.0, 1.0,  # Acceleration (gravity)
                    0.0, 0.0, 0.0,  # Gyroscope
                ]
                self.imu_outlet.push_sample(imu)

            t += 1 / 200  # 200 Hz
            time.sleep(1 / 200)

    def stop(self):
        """Stop generating data."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        print(f"Mock streamer stopped. Generated {self.sample_count} samples.")

    @property
    def is_connected(self) -> bool:
        return self._running


def main():
    """Run the Myo streamer from command line."""
    import argparse

    parser = argparse.ArgumentParser(description="Stream Myo data to LSL")
    parser.add_argument("--mock", action="store_true", help="Use mock data (no hardware)")
    parser.add_argument("--stream", default="Myo", help="Stream name prefix")
    parser.add_argument("--duration", type=int, default=0, help="Duration in seconds (0=infinite)")
    parser.add_argument("--mode", default="raw", choices=["raw", "filtered", "preprocessed"],
                        help="EMG mode")
    args = parser.parse_args()

    if args.mock:
        print("\n=== Mock Myo Streamer ===")
        print("Generating synthetic EMG data for testing\n")
        streamer = MockMyoStreamer(stream_name=args.stream)
    else:
        print("\n=== Myo LSL Streamer (pyomyo) ===")
        print("Using direct Bluetooth connection - no SDK needed!\n")
        streamer = MyoLSLStreamer(stream_name=args.stream, emg_mode_setting=args.mode)

    try:
        streamer.start()

        if args.duration > 0:
            print(f"\nStreaming for {args.duration} seconds...")
            time.sleep(args.duration)
        else:
            print("\nStreaming... Press Ctrl+C to stop\n")
            while True:
                time.sleep(1)
                print(f"  Samples: {streamer.sample_count}", end='\r')

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()