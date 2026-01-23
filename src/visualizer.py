"""
BioRobotics Lab 1 - Real-time EMG Visualizer
=============================================

A Python-based visualizer for EMG and other biosignals streamed via LSL.
This replaces the need for BioCapture or other proprietary software.

Features:
- Real-time multi-channel plotting
- Adjustable time window
- Signal quality indicators
- Recording capabilities
- Works without admin rights

Usage:
    python visualizer.py                    # Auto-detect streams
    python visualizer.py --stream "Myo"     # Connect to specific stream

Author: BioRobotics Course
Updated: 2025
"""

import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

# Check for PyQt6/PyQtGraph availability
try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QComboBox, QLabel, QSpinBox, QStatusBar,
        QGroupBox, QGridLayout, QCheckBox, QFileDialog, QMessageBox
    )
    from PyQt6.QtCore import QTimer, Qt
    from PyQt6.QtGui import QFont
    import pyqtgraph as pg
    HAS_GUI = True
except ImportError:
    HAS_GUI = False
    print("Warning: PyQt6 or pyqtgraph not available. GUI disabled.")

try:
    import pylsl
    HAS_LSL = True
except ImportError:
    HAS_LSL = False
    print("Warning: pylsl not available. LSL functionality disabled.")


@dataclass
class VisualizerConfig:
    """Configuration for the visualizer."""
    window_seconds: float = 5.0  # Time window to display
    update_rate_hz: float = 30.0  # Display update rate
    max_channels: int = 16  # Maximum channels to display
    dark_mode: bool = True


class SignalBuffer:
    """Thread-safe buffer for streaming signal data."""
    
    def __init__(self, n_channels: int, max_samples: int = 10000):
        self.n_channels = n_channels
        self.max_samples = max_samples
        self.data = [deque(maxlen=max_samples) for _ in range(n_channels)]
        self.timestamps = deque(maxlen=max_samples)
        self._lock = threading.Lock()
    
    def add_samples(self, samples: np.ndarray, timestamps: np.ndarray):
        """Add new samples to the buffer."""
        with self._lock:
            for i, sample in enumerate(samples):
                for ch in range(min(len(sample), self.n_channels)):
                    self.data[ch].append(sample[ch])
                if i < len(timestamps):
                    self.timestamps.append(timestamps[i])
    
    def get_data(self, n_samples: int = None) -> tuple[np.ndarray, np.ndarray]:
        """Get data from the buffer."""
        with self._lock:
            if n_samples is None:
                n_samples = len(self.timestamps)
            
            data = np.zeros((n_samples, self.n_channels))
            for ch in range(self.n_channels):
                ch_data = list(self.data[ch])[-n_samples:]
                data[:len(ch_data), ch] = ch_data
            
            timestamps = np.array(list(self.timestamps)[-n_samples:])
            return data, timestamps
    
    def clear(self):
        """Clear the buffer."""
        with self._lock:
            for ch_data in self.data:
                ch_data.clear()
            self.timestamps.clear()


class LSLStreamReader(threading.Thread):
    """Background thread for reading LSL data."""
    
    def __init__(self, stream_name: str, buffer: SignalBuffer):
        super().__init__(daemon=True)
        self.stream_name = stream_name
        self.buffer = buffer
        self.running = False
        self.inlet = None
        self.sample_count = 0
        self.error = None
    
    def run(self):
        """Main thread loop."""
        self.running = True
        self.sample_count = 0
        
        try:
            # Find and connect to stream
            print(f"Looking for stream: {self.stream_name}")
            try:
                streams = pylsl.resolve_byprop("name", self.stream_name, timeout=5.0)
            except TypeError:
                streams = pylsl.resolve_byprop("name", self.stream_name, 1, 5.0)
            
            if not streams:
                self.error = f"Stream '{self.stream_name}' not found"
                self.running = False
                return
            
            self.inlet = pylsl.StreamInlet(streams[0], max_buflen=360)
            print(f"Connected to {self.stream_name}")
            
            # Read data
            while self.running:
                samples, timestamps = self.inlet.pull_chunk(timeout=0.1)
                if samples:
                    self.buffer.add_samples(np.array(samples), np.array(timestamps))
                    self.sample_count += len(samples)
        
        except Exception as e:
            self.error = str(e)
        
        finally:
            self.running = False
    
    def stop(self):
        """Stop the reader thread."""
        self.running = False


if HAS_GUI and HAS_LSL:
    
    class EMGVisualizer(QMainWindow):
        """Main visualizer window."""
        
        def __init__(self, config: VisualizerConfig = None):
            super().__init__()
            self.config = config or VisualizerConfig()
            
            # State
            self.stream_reader: Optional[LSLStreamReader] = None
            self.buffer: Optional[SignalBuffer] = None
            self.recording = False
            self.record_data = []
            self.record_timestamps = []
            
            # Setup UI
            self.setup_ui()
            self.setup_plots()
            
            # Update timer
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_plots)
            
            # Refresh available streams
            self.refresh_streams()
        
        def setup_ui(self):
            """Create the user interface."""
            self.setWindowTitle("BioRobotics EMG Visualizer")
            self.setGeometry(100, 100, 1200, 800)
            
            # Apply dark theme if configured
            if self.config.dark_mode:
                pg.setConfigOption('background', '#1e1e1e')
                pg.setConfigOption('foreground', '#ffffff')
            
            # Central widget
            central = QWidget()
            self.setCentralWidget(central)
            layout = QVBoxLayout(central)
            
            # === Control Panel ===
            control_group = QGroupBox("Controls")
            control_layout = QGridLayout(control_group)
            
            # Stream selection
            control_layout.addWidget(QLabel("Stream:"), 0, 0)
            self.stream_combo = QComboBox()
            self.stream_combo.setMinimumWidth(200)
            control_layout.addWidget(self.stream_combo, 0, 1)
            
            self.refresh_btn = QPushButton("🔄 Refresh")
            self.refresh_btn.clicked.connect(self.refresh_streams)
            control_layout.addWidget(self.refresh_btn, 0, 2)
            
            self.connect_btn = QPushButton("▶ Connect")
            self.connect_btn.clicked.connect(self.toggle_connection)
            control_layout.addWidget(self.connect_btn, 0, 3)
            
            # Time window
            control_layout.addWidget(QLabel("Window (s):"), 0, 4)
            self.window_spin = QSpinBox()
            self.window_spin.setRange(1, 30)
            self.window_spin.setValue(int(self.config.window_seconds))
            self.window_spin.valueChanged.connect(self.update_window)
            control_layout.addWidget(self.window_spin, 0, 5)
            
            # Recording
            self.record_btn = QPushButton("⏺ Record")
            self.record_btn.clicked.connect(self.toggle_recording)
            self.record_btn.setEnabled(False)
            control_layout.addWidget(self.record_btn, 0, 6)
            
            self.save_btn = QPushButton("💾 Save")
            self.save_btn.clicked.connect(self.save_recording)
            self.save_btn.setEnabled(False)
            control_layout.addWidget(self.save_btn, 0, 7)
            
            layout.addWidget(control_group)
            
            # === Plot Area ===
            self.plot_widget = pg.GraphicsLayoutWidget()
            layout.addWidget(self.plot_widget, stretch=1)
            
            # === Status Bar ===
            self.status_bar = QStatusBar()
            self.setStatusBar(self.status_bar)
            self.status_label = QLabel("Not connected")
            self.sample_label = QLabel("Samples: 0")
            self.status_bar.addWidget(self.status_label)
            self.status_bar.addPermanentWidget(self.sample_label)
        
        def setup_plots(self, n_channels: int = 8):
            """Setup the plot widgets."""
            self.plot_widget.clear()
            self.plots = []
            self.curves = []
            
            for i in range(n_channels):
                if i > 0:
                    self.plot_widget.nextRow()
                
                plot = self.plot_widget.addPlot(title=f"Channel {i+1}")
                plot.setLabel('left', 'Amplitude')
                plot.setLabel('bottom', 'Time (s)')
                plot.showGrid(x=True, y=True, alpha=0.3)
                plot.setYRange(-1, 1)
                
                curve = plot.plot(pen=pg.mkPen(color=pg.intColor(i, hues=n_channels), width=1))
                
                self.plots.append(plot)
                self.curves.append(curve)
        
        def refresh_streams(self):
            """Scan for available LSL streams."""
            self.stream_combo.clear()
            self.status_label.setText("Scanning for streams...")
            QApplication.processEvents()
            
            try:
                # Parameter name varies by pylsl version
                try:
                    streams = pylsl.resolve_streams(wait_time=2.0)
                except TypeError:
                    streams = pylsl.resolve_streams(2.0)
                
                for stream in streams:
                    name = stream.name()
                    stype = stream.type()
                    n_ch = stream.channel_count()
                    rate = stream.nominal_srate()
                    self.stream_combo.addItem(
                        f"{name} ({stype}, {n_ch}ch, {rate}Hz)",
                        name
                    )
                
                if streams:
                    self.status_label.setText(f"Found {len(streams)} stream(s)")
                else:
                    self.status_label.setText("No streams found")
            
            except Exception as e:
                self.status_label.setText(f"Error: {e}")
        
        def toggle_connection(self):
            """Connect or disconnect from stream."""
            if self.stream_reader and self.stream_reader.running:
                self.disconnect_stream()
            else:
                self.connect_stream()
        
        def connect_stream(self):
            """Connect to the selected stream."""
            if self.stream_combo.currentIndex() < 0:
                QMessageBox.warning(self, "Error", "No stream selected")
                return
            
            stream_name = self.stream_combo.currentData()
            
            # Get stream info
            try:
                streams = pylsl.resolve_byprop("name", stream_name, timeout=2.0)
            except TypeError:
                streams = pylsl.resolve_byprop("name", stream_name, 1, 2.0)
            if not streams:
                QMessageBox.warning(self, "Error", f"Stream '{stream_name}' not found")
                return
            
            n_channels = min(streams[0].channel_count(), self.config.max_channels)
            sample_rate = streams[0].nominal_srate()
            
            # Setup buffer and plots
            max_samples = int(self.config.window_seconds * sample_rate * 2)
            self.buffer = SignalBuffer(n_channels, max_samples)
            self.setup_plots(n_channels)
            
            # Start reader thread
            self.stream_reader = LSLStreamReader(stream_name, self.buffer)
            self.stream_reader.start()
            
            # Start update timer
            interval_ms = int(1000 / self.config.update_rate_hz)
            self.timer.start(interval_ms)
            
            # Update UI
            self.connect_btn.setText("⏹ Disconnect")
            self.record_btn.setEnabled(True)
            self.status_label.setText(f"Connected to {stream_name}")
        
        def disconnect_stream(self):
            """Disconnect from the current stream."""
            self.timer.stop()
            
            if self.stream_reader:
                self.stream_reader.stop()
                self.stream_reader.join(timeout=1.0)
                self.stream_reader = None
            
            self.connect_btn.setText("▶ Connect")
            self.record_btn.setEnabled(False)
            self.status_label.setText("Disconnected")
        
        def update_plots(self):
            """Update the plots with new data."""
            if not self.buffer:
                return
            
            # Get data
            data, timestamps = self.buffer.get_data()
            
            if len(timestamps) < 2:
                return
            
            # Calculate time axis relative to now
            t_now = timestamps[-1]
            t_rel = timestamps - t_now
            
            # Update sample count
            if self.stream_reader:
                self.sample_label.setText(f"Samples: {self.stream_reader.sample_count}")
            
            # Update each channel
            for i, curve in enumerate(self.curves):
                if i < data.shape[1]:
                    curve.setData(t_rel, data[:, i])
                    
                    # Auto-scale Y axis
                    if len(data) > 10:
                        ch_data = data[:, i]
                        y_range = max(abs(ch_data.min()), abs(ch_data.max()), 0.1) * 1.2
                        self.plots[i].setYRange(-y_range, y_range)
            
            # Recording
            if self.recording:
                self.record_data.append(data.copy())
                self.record_timestamps.append(timestamps.copy())
        
        def update_window(self, value):
            """Update the time window."""
            self.config.window_seconds = float(value)
            for plot in self.plots:
                plot.setXRange(-value, 0)
        
        def toggle_recording(self):
            """Start or stop recording."""
            if self.recording:
                self.recording = False
                self.record_btn.setText("⏺ Record")
                self.save_btn.setEnabled(True)
                self.status_label.setText("Recording stopped")
            else:
                self.record_data.clear()
                self.record_timestamps.clear()
                self.recording = True
                self.record_btn.setText("⏹ Stop")
                self.save_btn.setEnabled(False)
                self.status_label.setText("Recording...")
        
        def save_recording(self):
            """Save the recording to a file."""
            if not self.record_data:
                QMessageBox.warning(self, "Error", "No data to save")
                return
            
            filename, _ = QFileDialog.getSaveFileName(
                self, "Save Recording", "", "CSV Files (*.csv);;All Files (*)"
            )
            
            if filename:
                import pandas as pd
                
                # Concatenate all data
                all_data = np.vstack(self.record_data)
                all_timestamps = np.concatenate(self.record_timestamps)
                
                # Create DataFrame
                columns = [f"ch_{i+1}" for i in range(all_data.shape[1])]
                df = pd.DataFrame(all_data, columns=columns)
                df.insert(0, 'timestamp', all_timestamps[:len(df)])
                
                df.to_csv(filename, index=False)
                self.status_label.setText(f"Saved to {filename}")
        
        def closeEvent(self, event):
            """Handle window close."""
            self.disconnect_stream()
            event.accept()


def main():
    """Main entry point."""
    if not HAS_GUI:
        print("Error: GUI dependencies not available.")
        print("Install with: pip install PyQt6 pyqtgraph")
        return 1
    
    if not HAS_LSL:
        print("Error: pylsl not available.")
        print("Install with: pip install pylsl")
        return 1
    
    import argparse
    parser = argparse.ArgumentParser(description="EMG Visualizer")
    parser.add_argument("--stream", help="Stream name to connect to")
    parser.add_argument("--dark", action="store_true", default=True, help="Dark mode")
    parser.add_argument("--mock", action="store_true", help="Create internal mock stream for testing")
    args = parser.parse_args()
    
    # Start mock stream if requested
    mock_thread = None
    mock_outlet = None
    if args.mock:
        print("Starting internal mock EMG stream...")
        mock_outlet, mock_thread = create_mock_stream()
        time.sleep(0.5)  # Give stream time to register
        args.stream = "MockEMG_EMG"  # Auto-connect to mock stream
    
    app = QApplication(sys.argv)
    
    config = VisualizerConfig(dark_mode=args.dark)
    window = EMGVisualizer(config)
    window.show()
    
    # Auto-connect if stream specified
    if args.stream:
        # Refresh to find the stream first
        window.refresh_streams()
        # Find and select the stream
        for i in range(window.stream_combo.count()):
            if args.stream in window.stream_combo.itemText(i):
                window.stream_combo.setCurrentIndex(i)
                break
        window.connect_stream()
    
    result = app.exec()
    
    # Cleanup mock stream
    if mock_thread:
        mock_thread.do_run = False
        mock_thread.join(timeout=1.0)
    
    return result


def create_mock_stream():
    """Create an internal mock EMG stream for testing."""
    import threading
    
    # Create stream info
    info = pylsl.StreamInfo(
        name="MockEMG_EMG",
        type="EMG",
        channel_count=8,
        nominal_srate=200,
        channel_format=pylsl.cf_float32,
        source_id="MockEMG_internal"
    )
    
    outlet = pylsl.StreamOutlet(info)
    print(f"Created mock stream: MockEMG_EMG (8ch, 200Hz)")
    
    def generate_data(outlet):
        """Generate synthetic EMG data."""
        t = 0
        thread = threading.current_thread()
        while getattr(thread, 'do_run', True):
            # Generate 8 channels of synthetic EMG
            sample = []
            for ch in range(8):
                # Base noise
                val = np.random.normal(0, 5)
                
                # Add periodic bursts
                if np.sin(2 * np.pi * 0.3 * t + ch * np.pi / 4) > 0.6:
                    val += np.random.normal(50, 20)
                
                # Add 60Hz interference
                val += 3 * np.sin(2 * np.pi * 60 * t)
                sample.append(val)
            
            outlet.push_sample(sample)
            t += 1/200
            time.sleep(1/200)
    
    thread = threading.Thread(target=generate_data, args=(outlet,), daemon=True)
    thread.do_run = True
    thread.start()
    
    return outlet, thread


if __name__ == "__main__":
    sys.exit(main())
