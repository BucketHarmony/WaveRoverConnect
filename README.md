# WAVE ROVER Wi-Fi Controller

A Python-based controller for the [Waveshare WAVE ROVER](https://www.waveshare.com/wiki/WAVE_ROVER) that connects over Wi-Fi and runs a full motion verification suite.

## Features

- **STA Mode Support** — Control the rover while your computer stays connected to the internet
- **Full Motion Test Suite** — Automated testing of all motion primitives (forward, reverse, turns, arcs, speed ramps)
- **Safety First** — E-STOP on Ctrl+C, automatic stop on errors, heartbeat validation
- **Keepalive System** — Continuous command loop prevents rover timeout
- **Detailed Logging** — Timestamps, HTTP responses, command loop rate metrics
- **Interactive Prompts** — Visual confirmation of each motion primitive

## Requirements

- Python 3.10+
- Waveshare WAVE ROVER
- Wi-Fi network

## Installation

```bash
git clone https://github.com/yourusername/roverConnect.git
cd roverConnect
pip install -r requirements.txt
```

## Quick Start

### 1. Provision Rover to Your Wi-Fi (One-Time Setup)

Connect your computer to the rover's access point:
- **SSID:** `UGV`
- **Password:** `12345678`

Then run:

```bash
python wave_rover_controller.py provision-sta \
    --sta-ssid "YourWiFiName" \
    --sta-password "YourWiFiPassword"
```

### 2. Get Rover's IP Address

After provisioning, the rover reboots and connects to your Wi-Fi. Check the rover's OLED display for the **ST** line — this shows the assigned IP address.

### 3. Connect Back to Your Normal Wi-Fi

Disconnect from `UGV` and reconnect to your regular Wi-Fi network.

### 4. Test Connectivity

```bash
python wave_rover_controller.py ping --ip 192.168.1.100
```

### 5. Run Motion Test Suite

```bash
python wave_rover_controller.py run-motion-suite --ip 192.168.1.100
```

## Commands

### `provision-sta` — Configure Wi-Fi Connection

```bash
python wave_rover_controller.py provision-sta \
    --sta-ssid "YourWiFi" \
    --sta-password "password" \
    [--ip 192.168.4.1] \
    [--ap-ssid UGV] \
    [--ap-password 12345678]
```

### `ping` — Test Connectivity

```bash
python wave_rover_controller.py ping --ip <ROVER_IP>
```

### `run-motion-suite` — Full Motion Test

```bash
python wave_rover_controller.py run-motion-suite --ip <ROVER_IP> \
    [--speed 0.15] \
    [--duration 2.0] \
    [--settle-delay 1.0] \
    [--non-interactive]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--speed` | 0.15 | Test speed in m/s |
| `--duration` | 2.0 | Duration per motion primitive (seconds) |
| `--settle-delay` | 1.0 | Delay between primitives (seconds) |
| `--non-interactive` | false | Skip user prompts |

### `stop` — Emergency Stop

```bash
python wave_rover_controller.py stop --ip <ROVER_IP>
```

### `move` — Manual Motion Command

```bash
python wave_rover_controller.py move --ip <ROVER_IP> \
    --left 0.1 --right 0.1 \
    [--duration 2.0]
```

## Motion Test Suite

The test suite validates these motion primitives:

| Test | Left Speed | Right Speed | Description |
|------|------------|-------------|-------------|
| Stop | 0 | 0 | Baseline / recovery |
| Forward | +v | +v | Both wheels forward |
| Reverse | -v | -v | Both wheels backward |
| Spin CW | +v | -v | Rotate clockwise in place |
| Spin CCW | -v | +v | Rotate counter-clockwise in place |
| Arc Left | +0.5v | +v | Curve left while moving forward |
| Arc Right | +v | +0.5v | Curve right while moving forward |
| Speed Ramp | varies | varies | Gradual acceleration/deceleration |
| Heartbeat | - | - | Validates ~3s auto-stop safety |

## Technical Details

### HTTP JSON Interface

Commands are sent via HTTP GET to:
```
http://<rover_ip>/js?json=<url_encoded_json>
```

### Motion Control Command

```json
{"T": 1, "L": 0.15, "R": 0.15}
```

- `T`: Command type (1 = speed control)
- `L`: Left wheel velocity (m/s, negative = reverse)
- `R`: Right wheel velocity (m/s, negative = reverse)

### Keepalive System

The rover stops automatically if no command is received within ~3 seconds. The controller sends commands at 4 Hz (every 0.25s) to maintain motion.

### Safety Features

- **E-STOP:** Press Ctrl+C at any time to immediately stop the rover
- **Auto-stop on error:** HTTP failures trigger automatic stop
- **Heartbeat validation:** Test suite verifies the safety timeout works

## Network Modes

| Mode | IP Address | Use Case |
|------|------------|----------|
| AP Mode | 192.168.4.1 | Initial provisioning |
| STA Mode | OLED "ST" line | Normal operation |

In STA mode, your computer stays on your normal Wi-Fi network with internet access while controlling the rover.

## Troubleshooting

### Cannot connect to rover

1. Verify the IP address on the rover's OLED display
2. Ensure your computer is on the same network
3. Check firewall settings
4. Try `ping <rover_ip>` from command line

### Rover doesn't move

1. Ensure the rover is powered on and not in charging mode
2. Check battery level
3. Verify motors are enabled (check rover documentation)

### Rover stops unexpectedly

This is normal — the rover has a ~3 second safety timeout. The controller's keepalive loop should prevent this during normal operation. If it happens, check for network latency issues.

### Provisioning failed

1. Ensure you're connected to the `UGV` access point
2. Verify the SSID and password are correct
3. Check that your Wi-Fi network is 2.4 GHz (if applicable)

## References

- [WAVE ROVER Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [Sub-controller JSON Command Set](https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set)

## License

MIT License
