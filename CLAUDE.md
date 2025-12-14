# WAVE ROVER Controller - Project Context

## Overview

This is a Python CLI tool for controlling the Waveshare WAVE ROVER over Wi-Fi. It implements the rover's HTTP JSON interface for motion control and includes a comprehensive motion test suite.

## Key Files

- `wave_rover_controller.py` — Main CLI application with all functionality
- `requirements.txt` — Python dependencies (just `requests`)
- `README.md` — User documentation

## Architecture

The controller is organized into these main components:

### RoverClient
HTTP client that sends JSON commands to the rover via `GET /js?json=<command>`.

### MotionController
Handles motion commands with:
- Keepalive loop (4 Hz) to prevent rover timeout
- E-STOP functionality
- Thread-safe speed updates

### MotionTestSuite
Interactive test suite that validates all motion primitives:
- Forward, reverse, turns, arcs
- Speed ramping
- Heartbeat/safety validation

## Command Protocol

Commands use Waveshare's JSON format:
```json
{"T": 1, "L": <left_speed>, "R": <right_speed>}
```

- `T=1`: CMD_SPEED_CTRL
- `L`, `R`: Wheel velocities in m/s (negative = reverse)

Provisioning command:
```json
{"T": 404, "ap_ssid": "...", "ap_password": "...", "sta_ssid": "...", "sta_password": "..."}
```

## Safety

- Rover auto-stops if no command received within ~3 seconds
- E-STOP sends `{"T":1,"L":0,"R":0}` immediately
- All exceptions trigger automatic stop

## Testing Commands

```bash
# Test connectivity
python wave_rover_controller.py ping --ip <IP>

# Run motion suite
python wave_rover_controller.py run-motion-suite --ip <IP>

# Manual move
python wave_rover_controller.py move --ip <IP> -l 0.1 -r 0.1 -d 2
```

## References

- [WAVE ROVER Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [JSON Command Set](https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set)
