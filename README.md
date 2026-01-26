# BioRobotics Lab 1: Introduction to LSL and EMG Signals

## Overview

This lab introduces you to biosignal acquisition using the **Lab Streaming Layer (LSL)** protocol and **electromyography (EMG)** signals. You will learn to collect, visualize, process, and analyze EMG data from the Myo Armband and BioRadio 150.

## Learning Objectives

By completing this lab, you will:

1. **Understand LSL Architecture** - How data flows from devices through a network
2. **Collect EMG Data** - Use hardware devices and software to record muscle signals  
3. **Visualize Signals in Real-time** - Monitor signal quality during collection
4. **Process EMG Signals** - Apply filtering, rectification, and envelope extraction
5. **Extract Features** - Compute meaningful metrics for classification
6. **Demonstrate Proportional Control** - Use EMG to control on-screen elements

## Quick Start

### Prerequisites

- Anaconda or Miniconda installed
- Git installed
- (Optional) Myo Armband with MyoConnect or BioRadio 150

### Installation

```bash
# Clone the repository
git clone https://github.com/BioRobotics-Spring-2020/Lab_1.git
cd Lab_1

# Create conda environment (no admin rights needed)
conda env create -f environment.yml

# Activate environment
conda activate biorobotics
```

### Running the Lab

1. **Start the visualizer** (to view EMG signals):
   ```bash
   python src/visualizer.py
   ```

2. **Run the proportional control demo**:
   ```bash
   python src/proportional_control.py --mock  # Without hardware
   python src/proportional_control.py         # With hardware
   ```

3. **Open the analysis notebook**:
   ```bash
   jupyter lab notebooks/Lab1_EMG_Analysis.ipynb
   ```

## Repository Structure

```
Lab_1/
├── environment.yml          # Conda environment specification
├── README.md               # This file
├── src/
│   ├── lsl_utils.py        # LSL utilities (stream discovery, recording)
│   ├── visualizer.py       # Real-time EMG visualizer
│   ├── myo_interface.py    # Myo Armband interface (uses pyomyo)
│   ├── emg_processing.py   # Signal processing functions
│   └── proportional_control.py  # Control demo
├── notebooks/
│   └── Lab1_EMG_Analysis.ipynb  # Main analysis notebook
├── data/
│   └── (your recorded data)
└── docs/
    └── Lab1_Manual.pdf     # Lab manual (PDF)
```

## Lab Streaming Layer (LSL)

LSL is a protocol for streaming time-series data over a network. Key concepts:

- **Stream**: A source of data (e.g., EMG from Myo)
- **Outlet**: Publishes data to the network
- **Inlet**: Receives data from a stream
- **XDF**: File format for storing LSL recordings

### Architecture

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   BioRadio   │    │     Myo      │    │   Markers    │
│   (Device)   │    │  (Armband)   │    │  (Software)  │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────────────────────────────────────────────────┐
│              Lab Streaming Layer (Network)              │
└─────────────────────────────────────────────────────────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  Visualizer  │    │   Recorder   │    │   Analysis   │
└──────────────┘    └──────────────┘    └──────────────┘
```

## Hardware Setup

### Myo Armband

The Myo interface supports **two backends**:

| Backend | Dongle Required | Best For |
|---------|-----------------|----------|
| **dl-myo** (recommended) | No - uses native Bluetooth | Multiple students, modern setup |
| **pyomyo** (fallback) | Yes - blue USB dongle | Single device, proven reliability |

**Installation:**
```bash
# dl-myo (recommended - no dongle needed)
pip install dl-myo

# pyomyo (fallback - requires dongle)
pip install git+https://github.com/PerlinWarp/pyomyo.git
```

**IMPORTANT: Close MyoConnect before running!**

#### Basic Usage

```bash
# Auto-detect backend and connect
python src/myo_interface.py

# Test without hardware
python src/myo_interface.py --mock
```

#### EMG Modes
- `--mode raw` (default): 200Hz, unfiltered (-128 to 127)
- `--mode filtered`: 200Hz, filtered but not rectified
- `--mode preprocessed`: 50Hz, bandpass filtered + rectified

#### Multiple Myos / Multiple Students (dl-myo)

With dl-myo, students can connect to specific Myos by MAC address:

```bash
# Scan for all Myos in range
python src/myo_interface.py --scan

# Connect to specific Myo by MAC address
python src/myo_interface.py --mac D2:3B:85:94:32:8E
```

**Tip:** Label each Myo with its MAC address for easy identification!

#### Using the Dongle (pyomyo fallback)

If dl-myo doesn't work, you can fall back to pyomyo with the dongle:

```bash
# Force pyomyo backend
python src/myo_interface.py --backend pyomyo

# List available serial ports (to find dongle)
python src/myo_interface.py --list-ports

# Connect to specific dongle
python src/myo_interface.py --backend pyomyo --tty COM5
```

#### Rollback to pyomyo Only

If you need to use only pyomyo (the previous working version):
```bash
# A backup is available at:
cp src/myo_interface_pyomyo.py src/myo_interface.py
```

### BioRadio 150

1. Place electrodes on forearm
2. Connect to BioRadio harness
3. Plug in USB receiver
4. Open BioCapture software
5. Configure channels and start streaming

## Data Collection

### Using the Visualizer

```bash
python src/visualizer.py
```

1. Click "Refresh" to find available streams
2. Select your device stream
3. Click "Connect" to start visualization
4. Click "Record" to save data
5. Click "Save" to export to CSV

### Using Python Scripts

```python
from src.lsl_utils import LSLRecorder, discover_streams

# Find available streams
streams = discover_streams()

# Record data
recorder = LSLRecorder()
recorder.add_stream("Myo_EMG")
recorder.start()

# ... collect data ...

recorder.stop()
recorder.save("my_recording.csv")
```

## Signal Processing

The EMG processing pipeline:

1. **Bandpass Filter** (20-95 Hz): Removes low-frequency motion artifacts and high-frequency noise
2. **Notch Filter** (60 Hz): Removes power line interference
3. **Rectification**: Takes absolute value
4. **Envelope Extraction**: Smooths to get activation level

```python
from src.emg_processing import process_emg_pipeline

result = process_emg_pipeline(raw_emg, sample_rate=200)
# Returns: raw, filtered, rectified, envelope, rms
```

## Proportional Control Demo

This demo shows how EMG can control external systems (basis for myoelectric prosthetics):

```bash
python src/proportional_control.py --mock  # Test without hardware
```

- Contract your muscles to move the bar
- Try different channels and gain settings
- Use "Target Tracking" mode to practice control precision

## Troubleshooting

### No Streams Found

- Ensure device is powered on and connected
- Check that device software (BioCapture for BioRadio) is running
- Try increasing the timeout: `discover_streams(timeout=5.0)`

### Myo Connection Issues

- **Close MyoConnect first!** pyomyo and MyoConnect cannot both access the Myo
- Make sure the Myo is charged and powered on
- The Myo should vibrate when pyomyo connects
- Try: `python src/myo_interface.py --mock` to test without hardware

### Poor Signal Quality

- Check electrode placement and contact
- Ensure skin is clean and dry
- Adjust bandpass filter frequencies
- Check for sources of electrical interference

### Import Errors

- Make sure conda environment is activated: `conda activate biorobotics`
- Reinstall packages: `pip install -r requirements.txt`

## References

- Lab Streaming Layer: https://labstreaminglayer.org/
- pylsl Documentation: https://github.com/labstreaminglayer/pylsl
- EMG Signal Processing: De Luca, C.J. (1997) "The Use of Surface Electromyography in Biomechanics"

## License

This lab material is for educational use in the BioRobotics course.
