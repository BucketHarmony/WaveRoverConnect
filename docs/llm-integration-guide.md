# WAVE ROVER LLM Integration Guide

A comprehensive reference for LLMs to connect to and command the Waveshare WAVE ROVER. This document captures all empirical findings, API details, and practical knowledge needed for successful integration.

## Quick Start for LLMs

```python
import requests
from urllib.parse import quote
import json

# Basic command sending
def send_command(ip: str, command: dict) -> dict:
    """Send a JSON command to the rover."""
    url = f"http://{ip}/js?json={quote(json.dumps(command))}"
    response = requests.get(url, timeout=2.0)
    return json.loads(response.text)

# Example: Move forward
send_command("192.168.4.115", {"T": 1, "L": 0.15, "R": 0.15})

# Example: Stop
send_command("192.168.4.115", {"T": 1, "L": 0, "R": 0})
```

## Network Architecture

### Connection Modes

The rover supports three network modes:

| Mode | Value | Description | Use Case |
|------|-------|-------------|----------|
| AP Only | 1 | Rover creates its own WiFi network | Initial setup, no router needed |
| STA Only | 2 | Rover connects to existing WiFi | Production use, rover on home network |
| AP+STA | 3 | Both modes simultaneously | **Recommended** - flexibility |

### Default AP Credentials
- **SSID:** `UGV`
- **Password:** `12345678`
- **Default IP:** `192.168.4.1` (when connected to rover's AP)

### Finding the Rover's IP

**Method 1: OLED Display**
- The rover's OLED shows two IP lines:
  - `AP: 192.168.4.1` - Access Point IP (always this when AP enabled)
  - `ST: 192.168.x.x` - Station IP (assigned by your router)

**Method 2: Network Scan**
```bash
python wave_rover_controller.py scan --subnet 192.168.1
```

**Method 3: Router DHCP Table**
- Check your router's connected devices list
- Look for device named "ESP32" or similar

## HTTP JSON API

### Endpoint Format

All commands use HTTP GET to a single endpoint:
```
http://<rover_ip>/js?json=<url_encoded_json>
```

### URL Encoding

Commands must be URL-encoded. Example:
```python
from urllib.parse import quote
command = {"T": 1, "L": 0.15, "R": 0.15}
url = f"http://192.168.4.115/js?json={quote(json.dumps(command))}"
# Result: http://192.168.4.115/js?json=%7B%22T%22%3A1%2C%22L%22%3A0.15%2C%22R%22%3A0.15%7D
```

### Response Format

All responses are JSON. The `T` field in the response indicates the response type (usually 1000 + request T value).

## Motion Control Commands

### T:1 - Velocity Control (Primary Motion Command)

```json
{"T": 1, "L": <left_speed>, "R": <right_speed>}
```

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| T | int | 1 | Command type identifier |
| L | float | -1.0 to 1.0 | Left wheel velocity (m/s) |
| R | float | -1.0 to 1.0 | Right wheel velocity (m/s) |

**Response:**
```json
{"T": 1001, "L": 0.15, "R": 0.15}
```

### Motion Examples

```json
// Forward
{"T": 1, "L": 0.15, "R": 0.15}

// Reverse
{"T": 1, "L": -0.15, "R": -0.15}

// Spin clockwise (in place)
{"T": 1, "L": 0.35, "R": -0.35}

// Spin counter-clockwise
{"T": 1, "L": -0.35, "R": 0.35}

// Arc left (curve while moving forward)
{"T": 1, "L": 0.08, "R": 0.40}

// Arc right
{"T": 1, "L": 0.40, "R": 0.08}

// Stop
{"T": 1, "L": 0, "R": 0}
```

## Status & Sensor Commands

### T:130 - Chassis/Battery Feedback

**Request:**
```json
{"T": 130}
```

**Response:**
```json
{
    "T": 1001,
    "L": 0,           // Current left wheel speed (m/s)
    "R": 0,           // Current right wheel speed (m/s)
    "r": 0.58,        // Roll angle (degrees)
    "p": -0.71,       // Pitch angle (degrees)
    "y": -173.17,     // Yaw angle (degrees)
    "temp": 32.22,    // Temperature (Celsius)
    "v": 11.20        // Battery voltage (V)
}
```

**Voltage Reference:**
| Voltage | Status |
|---------|--------|
| 12.6V | Fully charged |
| 11.1V | Nominal |
| 10.5V | Low - consider recharging |
| 10.0V | Critical - recharge immediately |

### T:126 - IMU Sensor Data

**Request:**
```json
{"T": 126}
```

**Response:**
```json
{
    "T": 1002,
    "r": 0.89,        // Roll (degrees)
    "p": -0.74,       // Pitch (degrees)
    "y": -173.05,     // Yaw (degrees)
    "ax": 11.89,      // Accelerometer X (mg, 1000 = 1g)
    "ay": 14.07,      // Accelerometer Y (mg)
    "az": 983.41,     // Accelerometer Z (mg) - ~1000 when flat
    "gx": 0.12,       // Gyroscope X (degrees/second)
    "gy": 0.13,       // Gyroscope Y (degrees/second)
    "gz": 0.22,       // Gyroscope Z (degrees/second)
    "mx": 39,         // Magnetometer X (raw)
    "my": -7,         // Magnetometer Y (raw)
    "mz": -69,        // Magnetometer Z (raw)
    "temp": 32.22     // Temperature (Celsius)
}
```

### T:405 - WiFi Information

**Request:**
```json
{"T": 405}
```

**Response:**
```json
{
    "ip": "192.168.4.115",
    "rssi": -41,              // Signal strength (dBm)
    "wifi_mode_on_boot": 3    // Current WiFi mode setting
}
```

**RSSI Reference:**
| RSSI (dBm) | Quality |
|------------|---------|
| -30 to -50 | Excellent |
| -50 to -60 | Good |
| -60 to -70 | Fair |
| Below -70 | Weak |

## WiFi Configuration Commands

### T:401 - Set WiFi Mode

```json
{"T": 401, "cmd": <mode>}
```

| Mode | Description |
|------|-------------|
| 0 | WiFi off |
| 1 | AP only |
| 2 | STA only |
| 3 | AP+STA (recommended) |

### T:404 - Set WiFi Credentials

```json
{
    "T": 404,
    "ap_ssid": "UGV",
    "ap_password": "12345678",
    "sta_ssid": "YourHomeWiFi",
    "sta_password": "YourPassword"
}
```

### T:600 - Reboot ESP32

```json
{"T": 600}
```

Note: Device may not respond as it reboots immediately.

## Critical: Heartbeat/Safety Timeout

**THE ROVER HAS A BUILT-IN SAFETY TIMEOUT**

- **Timeout Duration:** ~3 seconds
- **Behavior:** If no motion command is received within the timeout, the rover automatically stops
- **Purpose:** Prevents runaway if communication is lost

### Implementing Keepalive

For continuous motion, you MUST send commands at regular intervals:

```python
import threading
import time

class MotionController:
    def __init__(self, ip):
        self.ip = ip
        self.left_speed = 0
        self.right_speed = 0
        self.running = False

    def keepalive_loop(self):
        """Send commands at 4 Hz to maintain motion."""
        while self.running:
            send_command(self.ip, {
                "T": 1,
                "L": self.left_speed,
                "R": self.right_speed
            })
            time.sleep(0.25)  # 4 Hz = every 250ms

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.keepalive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.left_speed = 0
        self.right_speed = 0
        send_command(self.ip, {"T": 1, "L": 0, "R": 0})
```

**Recommended keepalive interval:** 250ms (4 Hz)

## Motion Characteristics & Power Thresholds

### CRITICAL: Minimum Power Requirements

The rover has **minimum power thresholds** that must be exceeded for motion to occur. These vary by motion type due to different friction and torque requirements.

| Motion Type | Minimum Speed | Recommended | Why |
|-------------|---------------|-------------|-----|
| Forward/Reverse | ~0.10 m/s | 0.15 m/s | Lowest friction |
| Spin (in-place) | ~0.30 m/s | 0.35 m/s | Must overcome wheel scrub |
| Arc (curved path) | ~0.35 m/s (fast wheel) | 0.40 m/s | Needs speed differential |

### Why Turns Need More Power

**Spin turns (rotate in place):**
- Both wheel sets must overcome static friction simultaneously
- Wheels must scrub sideways against the ground
- Mecanum/omni wheels have higher rotational resistance

**Arc turns (curved path):**
- One wheel moves faster than the other
- The slow wheel may stall if below its friction threshold
- Small speed differentials result in nearly straight motion

### Speed Differential for Arcs

For reliable arc turns, the **ratio** matters more than absolute speed:

```python
# POOR arc (will go nearly straight):
L = 0.12  # m/s
R = 0.15  # m/s
# Ratio: 0.8 - too close

# GOOD arc (visible curve):
L = 0.08  # m/s (just above stall)
R = 0.40  # m/s (high speed)
# Ratio: 0.2 - large differential
```

### Recommended Test Speeds

```python
# Forward/reverse motion
forward_speed = 0.15  # m/s

# Spin turns (in place rotation)
spin_speed = 0.35     # m/s per wheel, opposite directions

# Arc turns (curved path)
arc_fast = 0.40       # m/s (outer/fast wheel)
arc_slow = 0.08       # m/s (inner/slow wheel)
```

## Complete Command Reference

| T Value | Command | Description |
|---------|---------|-------------|
| 1 | CMD_SPEED_CTRL | Velocity control (primary motion) |
| 126 | CMD_GET_IMU_DATA | Get IMU sensor readings |
| 130 | CMD_BASE_FEEDBACK | Get chassis/battery info |
| 302 | CMD_GET_MAC_ADDRESS | Get device MAC address |
| 401 | CMD_WIFI_MODE | Set WiFi mode on boot |
| 404 | CMD_WIFI_CONFIG | Set WiFi credentials |
| 405 | CMD_WIFI_INFO | Get current WiFi info |
| 600 | CMD_REBOOT | Reboot ESP32 |
| 601 | CMD_FREE_FLASH_SPACE | Get available flash storage |

## Example: Complete Motion Sequence

```python
import requests
import json
import time
import threading
from urllib.parse import quote

class WaveRover:
    def __init__(self, ip: str):
        self.ip = ip
        self.base_url = f"http://{ip}"
        self._speed_left = 0
        self._speed_right = 0
        self._keepalive_running = False
        self._keepalive_thread = None

    def send(self, command: dict) -> dict:
        """Send command and return parsed response."""
        url = f"{self.base_url}/js?json={quote(json.dumps(command))}"
        response = requests.get(url, timeout=2.0)
        return json.loads(response.text)

    def _keepalive_loop(self):
        while self._keepalive_running:
            try:
                self.send({"T": 1, "L": self._speed_left, "R": self._speed_right})
            except Exception as e:
                print(f"Keepalive error: {e}")
                self.stop()
                break
            time.sleep(0.25)

    def start_motion(self):
        """Start the keepalive thread."""
        if self._keepalive_thread and self._keepalive_thread.is_alive():
            return
        self._keepalive_running = True
        self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._keepalive_thread.start()

    def stop_motion(self):
        """Stop the keepalive thread."""
        self._keepalive_running = False
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=1.0)

    def set_speed(self, left: float, right: float):
        """Set wheel speeds (used by keepalive loop)."""
        self._speed_left = left
        self._speed_right = right

    def stop(self):
        """Emergency stop."""
        self.stop_motion()
        self._speed_left = 0
        self._speed_right = 0
        for _ in range(3):  # Send multiple times for reliability
            try:
                self.send({"T": 1, "L": 0, "R": 0})
                return True
            except:
                pass
        return False

    def forward(self, speed: float = 0.15, duration: float = 2.0):
        """Move forward for specified duration."""
        self.set_speed(speed, speed)
        self.start_motion()
        time.sleep(duration)
        self.stop()

    def spin(self, speed: float = 0.35, clockwise: bool = True, duration: float = 2.0):
        """Spin in place."""
        if clockwise:
            self.set_speed(speed, -speed)
        else:
            self.set_speed(-speed, speed)
        self.start_motion()
        time.sleep(duration)
        self.stop()

    def arc(self, fast_speed: float = 0.4, slow_speed: float = 0.08,
            turn_left: bool = True, duration: float = 2.0):
        """Arc turn while moving forward."""
        if turn_left:
            self.set_speed(slow_speed, fast_speed)
        else:
            self.set_speed(fast_speed, slow_speed)
        self.start_motion()
        time.sleep(duration)
        self.stop()

    def get_battery(self) -> float:
        """Get current battery voltage."""
        response = self.send({"T": 130})
        return response.get("v", 0)

    def get_imu(self) -> dict:
        """Get IMU sensor data."""
        return self.send({"T": 126})

    def ping(self) -> bool:
        """Test connectivity."""
        try:
            self.send({"T": 1, "L": 0, "R": 0})
            return True
        except:
            return False


# Usage example
if __name__ == "__main__":
    rover = WaveRover("192.168.4.115")

    if not rover.ping():
        print("Cannot connect to rover!")
        exit(1)

    print(f"Battery: {rover.get_battery()}V")

    # Move forward 2 seconds
    rover.forward(speed=0.15, duration=2.0)

    # Spin clockwise 2 seconds
    rover.spin(speed=0.35, clockwise=True, duration=2.0)

    # Arc left 2 seconds
    rover.arc(turn_left=True, duration=2.0)

    # Always stop at the end
    rover.stop()
```

## Hardware Reference

- **Model:** Waveshare WAVE ROVER
- **Controller:** ESP32
- **Drive:** Differential drive with 4 mecanum/omni wheels
- **Communication:** WiFi (2.4 GHz)
- **Battery:** 3S LiPo (~11.1V nominal)
- **Sensors:** IMU (accelerometer, gyroscope, magnetometer), temperature

## Additional Resources

- [Waveshare WAVE ROVER Wiki](https://www.waveshare.com/wiki/WAVE_ROVER)
- [JSON Command Set Documentation](https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set)

## Document History

- **Created:** December 2024
- **Based on:** Empirical testing with actual WAVE ROVER hardware
- **Controller version:** wave_rover_controller.py
