# TCP to Serial Antenna Control - GPredict Bridge

## Overview
Script that bridges GPredict satellite tracking software to antenna rotator controllers via serial communication. Enables automated antenna tracking for satellite passes.

## Origin

**Original Author**: Antoine Scherer (ISU IT Intern)  
**Original Repository**: [github.com/antscher/tcp_to_serial_antena_control](https://github.com/antscher/tcp_to_serial_antena_control)

This script is based on Antoine's work, with modifications for improved stability and error handling.

## Modifications from Original

### Key Changes by Harry Tabi Ndip (Leanspace)

**1. Master/Slave Initialization Order (Critical Fix)**
- **Original**: Both serial ports initialized simultaneously
- **Modified**: Sequential initialization - Azimuth (Master) first, then Elevation (Slave)
- **Impact**: Fixed hardware communication errors and improved stability
- **Reason**: Hardware requires master controller to be ready before slave initialization

**2. Enhanced Error Handling**
- **Added**: `serial.SerialException` catching in reader threads
- **Added**: Thread exit on serial errors (prevents zombie threads)
- **Improved**: More descriptive error messages with context

**3. CPU Usage Optimization**
- **Added**: `time.sleep(0.1)` in reader threads
- **Impact**: Prevents high CPU usage from tight loops
- **Original**: No delay, caused unnecessary CPU consumption

**4. Platform Adaptation**
- **Original**: Windows COM ports (`COM26`, `COM7`)
- **Modified**: Linux device nodes (`/dev/ttyUSB0`, `/dev/ttyUSB1`)
- **Impact**: Compatible with Linux-based ground station

**5. Code Cleanup**
- **Removed**: Commented-out code blocks
- **Added**: Comprehensive docstrings
- **Improved**: Code documentation and comments

### Detailed Changelog

```diff
# Main initialization order (CRITICAL FIX)
- Initialize both ports simultaneously
+ Sequential initialization with explicit master/slave order
+ Print statements for initialization tracking

# Serial reader functions
- Basic error handling
+ SerialException catching
+ Thread termination on errors
+ CPU optimization with sleep delay

# Platform configuration
- SERIAL_AZ_PORT = "COM26"
- SERIAL_EL_PORT = "COM7"
+ SERIAL_AZ_PORT = "/dev/ttyUSB0"  # Master
+ SERIAL_EL_PORT = "/dev/ttyUSB1"  # Slave
```

## Purpose

Enables GPredict to control antenna rotators for automated satellite tracking:
- **GPredict** calculates satellite position
- **TCP Bridge** (this script) receives tracking commands
- **Serial Controllers** move azimuth and elevation motors
- **Real-time tracking** follows satellite across the sky

## Architecture

```
┌─────────────────┐
│    GPredict     │ Satellite tracking software
│  (Tracking SW)  │
└────────┬────────┘
         │ TCP :4533
         │ Rotctld Protocol
         ↓
┌─────────────────┐
│  TCP Server     │ This script
│  (Bridge)       │
└───┬─────────┬───┘
    │         │
    │ Serial  │ Serial
    │ USB0    │ USB1
    ↓         ↓
┌─────────┐ ┌─────────┐
│Azimuth  │ │Elevation│ Antenna rotator controllers
│ (Master)│ │ (Slave) │
└─────────┘ └─────────┘
    │         │
    └────┬────┘
         ↓
   Antenna Rotator
```

## Features

### GPredict Integration
- **TCP Server**: Port 4533 (rotctld protocol)
- **Commands**: 
  - `p` - Query current position
  - `P <az> <el>` - Set new position
  - `S` - Stop
  - `q` - Quit

### Dual Serial Control
- **Azimuth Controller**: Master (`/dev/ttyUSB0`)
- **Elevation Controller**: Slave (`/dev/ttyUSB1`)
- **Baud Rate**: 9600
- **Protocol**: Custom rotator protocol with `A=` and `E=` feedback

### Thread-Safe Operation
- **Separate threads** for azimuth and elevation feedback reading
- **Thread locks** for shared position variables
- **Daemon threads** for clean shutdown

### Error Handling
- Serial communication error recovery
- Malformed data parsing protection
- Connection loss detection
- Graceful thread termination

## Configuration

### Serial Ports
Update in script header:
```python
SERIAL_AZ_PORT = "/dev/ttyUSB0"   # Azimuth controller (Master)
SERIAL_EL_PORT = "/dev/ttyUSB1"   # Elevation controller (Slave)
SERIAL_BAUD = 9600
```

### TCP Server
```python
TCP_HOST = "localhost"  # GPredict runs on same machine
TCP_PORT = 4533         # Standard rotctld port
```

## Usage

### Starting the Bridge

```bash
python3 scripts/tcp_to_serial_antenna_control.py
```

**Expected output:**
```
[INFO] Initializing controllers in Master/Slave order...
[INFO] Opening Azimuth port...
[INFO] Azimuth port opened.
[INFO] Opening Elevation port...
[INFO] Elevation port opened.
Waiting for Gpredict connection on localhost:4533...
```

### Configuring GPredict

1. **Open GPredict** → Preferences → Interfaces → Rotators
2. **Add New Rotator**:
   - **Name**: Ground Station Rotator
   - **Host**: localhost
   - **Port**: 4533
   - **Type**: ROT2PROG (or rotctld compatible)
3. **Enable tracking** for satellite pass
4. **Engage rotator control** during satellite tracking

### Testing

**Manual testing with telnet:**
```bash
telnet localhost 4533
p                    # Query position
P 45.0 30.0         # Set position to Az=45°, El=30°
```

## Hardware Requirements

- Antenna rotator with azimuth and elevation control
- Serial controllers supporting command format:
  - Commands: `Axxx.x\r` (azimuth), `Exxx.x\r` (elevation)
  - Feedback: `A=xxx.x` (azimuth), `E=xxx.x` (elevation)
- USB-to-Serial adapters (or built-in serial ports)
- Linux system with `/dev/ttyUSB*` device nodes

## Troubleshooting

### Serial Port Not Found
```bash
# List available serial ports
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1

# Or add user to dialout group
sudo usermod -a -G dialout $USER
```

### Controllers Not Responding
- Verify master/slave initialization order (script handles this)
- Check baud rate matches controller settings
- Verify serial cable connections
- Test with serial terminal (minicom, screen)

### GPredict Connection Failed
- Ensure TCP port 4533 is not in use
- Check firewall settings
- Verify localhost resolves correctly
- Review script output for initialization errors

### Position Not Updating
- Check serial feedback messages in script output
- Verify controllers are sending `A=` and `E=` feedback
- Enable serial reader threads (uncommented in main())
- Check thread error messages

## Dependencies

```bash
pip install pyserial
```

**Python Modules:**
- `socket` (standard library)
- `time` (standard library)
- `threading` (standard library)
- `serial` (pyserial package)

## Integration with Ground Station

This script is part of the complete ground station automation system:

**Related Components:**
- **GPredict**: Satellite tracking and Doppler calculation
- **GNU Radio Decoders**: `funcube_gui.py`, `funcube_headless.py`
- **Antenna Rotator**: Physical hardware for antenna positioning

**Operational Flow:**
1. GPredict tracks satellite position
2. This script moves antenna to follow satellite
3. GNU Radio decoder receives signal at optimal antenna pointing
4. Telemetry forwarded to Leanspace via route containers

## Credits

**Original Development**: Antoine Scherer (ISU IT Intern)  
**Repository**: https://github.com/antscher/tcp_to_serial_antena_control

**Modifications**: Harry Tabi Ndip (Leanspace)  
**Key Improvements**: Master/slave initialization fix, enhanced error handling, CPU optimization

## License

Check original repository for license terms. Modifications are compatible with original licensing.
