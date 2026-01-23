"""
BioRobotics Lab 1 - Myo Armband Interface
==========================================

This module provides an LSL interface for the Myo Armband using pyomyo.
pyomyo communicates directly via Bluetooth - NO SDK or MyoConnect needed!

IMPORTANT: Close MyoConnect before running this script!
           They cannot both access the Myo at the same time.

Requirements:
    pip install git+https://github.com/PerlinWarp/pyomyo.git

Channel Layout (EMG):
- EMG_1 through EMG_8: 8 sEMG channels around the forearm

EMG Modes:
- RAW: 200Hz, values -128 to 127 (unfiltered)
- FILTERED: 200Hz, filtered but not rectified  
- PREPROCESSED: 50Hz, bandpass filtered + rectified

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
from queue import Queue

import numpy as np

try:
    import pylsl
    HAS_LSL = True
except ImportError:
    HAS_LSL = False
    print("Warning: pylsl not available. Install with: pip install pylsl")

# Try to import pyomyo
HAS_PYOMYO = False
try:
    from pyomyo import Myo, emg_mode
    HAS_PYOMYO = True
except ImportError:
    print("pyomyo not available.")
    print("Install with: pip install git+https://github.com/PerlinWarp/pyomyo.git")


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class MyoData:
    """Container for Myo data."""
    emg: List[int] = field(default_factory=lambda: [0] * 8)
    timestamp: float = 0.0


# ============================================================================
# Myo LSL Streamer using pyomyo
# ============================================================================

class MyoStreamer:
    """
    Streams Myo Armband EMG data to LSL using pyomyo.
    
    pyomyo communicates directly via Bluetooth - no SDK or MyoConnect needed!
    IMPORTANT: Close MyoConnect before running this script!
    
    Example
    -------
    >>> streamer = MyoStreamer()
    >>> streamer.start()
    >>> time.sleep(10)  # Stream for 10 seconds
    >>> streamer.stop()
    
    Parameters
    ----------
    stream_name : str
        Base name for the LSL stream (default: "Myo")
    mode : str
        EMG mode: "raw" (200Hz), "filtered" (200Hz), or "preprocessed" (50Hz)
    """
    
    def __init__(self, stream_name: str = "Myo", mode: str = "raw"):
        if not HAS_PYOMYO:
            raise ImportError(
                "pyomyo not available.\n"
                "Install with: pip install git+https://github.com/PerlinWarp/pyomyo.git\n\n"
                "IMPORTANT: Close MyoConnect before running - they cannot both access the Myo!"
            )
        
        if not HAS_LSL:
            raise ImportError("pylsl not available. Install with: pip install pylsl")
        
        self.stream_name = stream_name
        self.mode_name = mode
        
        # Set EMG mode
        if mode == "raw":
            self.emg_mode = emg_mode.RAW
            self.sample_rate = 200
        elif mode == "filtered":
            self.emg_mode = emg_mode.FILTERED
            self.sample_rate = 200
        else:  # preprocessed
            self.emg_mode = emg_mode.PREPROCESSED
            self.sample_rate = 50
        
        self.myo = None
        self.sample_count = 0
        self._running = False
        self._thread = None
        self.emg_outlet = None
        self._emg_queue = Queue()
        
    def _setup_lsl_outlet(self):
        """Create LSL outlet for EMG data."""
        emg_info = pylsl.StreamInfo(
            name=f"{self.stream_name}_EMG",
            type='EMG',
            channel_count=8,
            nominal_srate=self.sample_rate,
            channel_format=pylsl.cf_int8 if self.emg_mode == emg_mode.RAW else pylsl.cf_float32,
            source_id=f'{self.stream_name}_EMG'
        )
        
        # Add channel descriptions
        desc = emg_info.desc()
        desc.append_child_value("manufacturer", "Thalmic Labs")
        desc.append_child_value("emg_mode", self.mode_name)
        channels = desc.append_child("channels")
        for i in range(8):
            ch = channels.append_child("channel")
            ch.append_child_value("label", f"EMG_{i+1}")
            ch.append_child_value("unit", "raw")
            ch.append_child_value("type", "EMG")
        
        self.emg_outlet = pylsl.StreamOutlet(emg_info)
        print(f"Created LSL outlet: {self.stream_name}_EMG ({self.sample_rate}Hz, {self.mode_name} mode)")
    
    def _emg_callback(self, emg, movement):
        """Called by pyomyo when EMG data is received."""
        # Queue the data for the LSL thread
        self._emg_queue.put(list(emg))
    
    def _lsl_thread(self):
        """Thread that pushes EMG data to LSL."""
        while self._running:
            try:
                # Get EMG data from queue (with timeout to allow checking _running)
                emg = self._emg_queue.get(timeout=0.1)
                self.emg_outlet.push_sample(emg)
                self.sample_count += 1
            except:
                pass  # Queue timeout, just continue
    
    def start(self):
        """Start streaming Myo data to LSL."""
        if self._running:
            print("Already running!")
            return
        
        print("=" * 50)
        print("Myo LSL Streamer (pyomyo)")
        print("=" * 50)
        print("\nIMPORTANT: Make sure MyoConnect is CLOSED!")
        print("           pyomyo connects directly via Bluetooth.\n")
        print("Connecting to Myo armband...")
        print("  - Make sure the Myo is charged and on your arm")
        print("  - The Myo should vibrate when connected\n")
        
        # Setup LSL outlet
        self._setup_lsl_outlet()
        
        # Create and connect Myo
        self.myo = Myo(mode=self.emg_mode)
        self.myo.connect()
        
        # Add EMG callback
        self.myo.add_emg_handler(self._emg_callback)
        
        self._running = True
        
        # Start LSL publishing thread
        self._lsl_thread_handle = threading.Thread(target=self._lsl_thread, daemon=True)
        self._lsl_thread_handle.start()
        
        # Start Myo run loop in separate thread
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        
        print("Myo streamer started!")
        print(f"Streaming EMG at {self.sample_rate}Hz ({self.mode_name} mode)")
    
    def _run_loop(self):
        """Main run loop for pyomyo."""
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
        
        print(f"\nStreamed {self.sample_count} EMG samples")
        print("Myo streamer stopped")
    
    @property
    def is_connected(self) -> bool:
        """Check if Myo is connected."""
        return self._running and self.myo is not None


# ============================================================================
# Mock Streamer for Testing
# ============================================================================

class MockMyoStreamer:
    """
    Mock Myo streamer for testing without hardware.
    Generates synthetic EMG data that looks realistic.
    """
    
    def __init__(self, stream_name: str = "MockMyo", sample_rate: int = 200):
        if not HAS_LSL:
            raise ImportError("pylsl not available. Install with: pip install pylsl")
        
        self.stream_name = stream_name
        self.sample_rate = sample_rate
        self.sample_count = 0
        self._running = False
        self._thread = None
        self.emg_outlet = None
        
    def _setup_lsl_outlet(self):
        """Create LSL outlet."""
        emg_info = pylsl.StreamInfo(
            name=f"{self.stream_name}_EMG",
            type='EMG',
            channel_count=8,
            nominal_srate=self.sample_rate,
            channel_format=pylsl.cf_int8,
            source_id=f'{self.stream_name}_EMG'
        )
        
        desc = emg_info.desc()
        desc.append_child_value("manufacturer", "Mock")
        channels = desc.append_child("channels")
        for i in range(8):
            ch = channels.append_child("channel")
            ch.append_child_value("label", f"EMG_{i+1}")
        
        self.emg_outlet = pylsl.StreamOutlet(emg_info)
        print(f"Created mock LSL outlet: {self.stream_name}_EMG ({self.sample_rate}Hz)")
    
    def start(self):
        """Start generating mock data."""
        if self._running:
            return
        
        print("=" * 50)
        print("Mock Myo Streamer")
        print("=" * 50)
        print("Generating synthetic EMG data for testing\n")
        
        self._setup_lsl_outlet()
        self._running = True
        self._thread = threading.Thread(target=self._generate_data, daemon=True)
        self._thread.start()
        print("Mock Myo streamer started")
    
    def _generate_data(self):
        """Generate synthetic EMG data."""
        t = 0
        dt = 1.0 / self.sample_rate
        
        while self._running:
            # Generate 8 channels of synthetic EMG
            emg = []
            for ch in range(8):
                # Base noise
                val = np.random.normal(0, 5)
                
                # Add periodic bursts (simulating muscle activation)
                # Different channels activate at different phases
                burst_phase = 2 * np.pi * 0.3 * t + ch * np.pi / 4
                if np.sin(burst_phase) > 0.6:
                    val += np.random.normal(50, 20)
                
                # Add subtle 60Hz interference
                val += 3 * np.sin(2 * np.pi * 60 * t)
                
                # Clamp to int8 range
                val = int(np.clip(val, -128, 127))
                emg.append(val)
            
            self.emg_outlet.push_sample(emg)
            self.sample_count += 1
            
            t += dt
            time.sleep(dt)
    
    def stop(self):
        """Stop generating data."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        print(f"\nMock streamer stopped. Generated {self.sample_count} samples.")
    
    @property
    def is_connected(self) -> bool:
        return self._running


# ============================================================================
# Command Line Interface
# ============================================================================

def main():
    """Run the Myo streamer from command line."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Stream Myo EMG data to LSL",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python myo_interface.py              # Stream with real Myo (raw mode)
  python myo_interface.py --mock       # Use mock data for testing
  python myo_interface.py --mode filtered   # Use filtered EMG mode
  python myo_interface.py --duration 30     # Stream for 30 seconds

IMPORTANT: Close MyoConnect before running! pyomyo connects directly via Bluetooth.
        """
    )
    parser.add_argument("--mock", action="store_true", 
                       help="Use mock data (no hardware needed)")
    parser.add_argument("--stream", default="Myo", 
                       help="LSL stream name prefix (default: Myo)")
    parser.add_argument("--mode", default="raw", 
                       choices=["raw", "filtered", "preprocessed"],
                       help="EMG mode: raw (200Hz), filtered (200Hz), preprocessed (50Hz)")
    parser.add_argument("--duration", type=int, default=0, 
                       help="Duration in seconds (0 = run until Ctrl+C)")
    args = parser.parse_args()
    
    if args.mock:
        streamer = MockMyoStreamer(stream_name=args.stream)
    else:
        if not HAS_PYOMYO:
            print("ERROR: pyomyo not installed!")
            print("Install with: pip install git+https://github.com/PerlinWarp/pyomyo.git")
            sys.exit(1)
        streamer = MyoStreamer(stream_name=args.stream, mode=args.mode)
    
    try:
        streamer.start()
        
        if args.duration > 0:
            print(f"\nStreaming for {args.duration} seconds...")
            for i in range(args.duration):
                time.sleep(1)
                print(f"  {i+1}/{args.duration}s - Samples: {streamer.sample_count}", end='\r')
            print()
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
