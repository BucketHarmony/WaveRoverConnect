# WAVE ROVER Motion Characteristics

This document captures empirical findings from testing the Waveshare WAVE ROVER's motion system.

## Overview

The WAVE ROVER uses differential drive with four mecanum/omni wheels. Motion is controlled via velocity commands to the left and right wheel sets (in m/s).

## Key Findings

### Minimum Power Thresholds

The rover has **minimum power thresholds** that must be exceeded for motion to occur. These thresholds vary by motion type due to different friction and torque requirements.

| Motion Type | Minimum Speed | Recommended Speed | Notes |
|-------------|---------------|-------------------|-------|
| Forward/Reverse | ~0.10 m/s | 0.15 m/s | Lowest friction, easiest motion |
| Spin (in-place turn) | ~0.30 m/s | 0.35 m/s | Requires overcoming wheel scrub friction |
| Arc (curved path) | ~0.35 m/s (fast wheel) | 0.40 m/s | Needs large speed differential |

### Why Turns Need More Power

**Spin turns (rotate in place):**
- Both wheel sets must overcome static friction simultaneously
- Wheels must scrub sideways against the ground
- Mecanum wheels have higher rotational resistance
- **Solution:** Use higher speeds (0.35+ m/s) for reliable in-place rotation

**Arc turns (curved forward motion):**
- One wheel set moves faster than the other
- The slow wheel may stall if below its friction threshold
- Small speed differentials result in nearly straight motion
- **Solution:** Use large speed differential (e.g., 0.40 vs 0.08 m/s)

### Speed Differential for Arcs

For reliable arc turns, the speed differential matters more than absolute speed:

```
# Poor arc (may go straight):
L=0.12 m/s, R=0.15 m/s  (ratio: 0.8)

# Good arc (visible curve):
L=0.08 m/s, R=0.40 m/s  (ratio: 0.2)
```

The slow wheel should be just above stall threshold (~0.08 m/s) while the fast wheel runs at higher speed (0.4+ m/s).

## Command Reference

### Motion Control Command

```json
{"T": 1, "L": <left_speed>, "R": <right_speed>}
```

- `T`: Command type (1 = velocity control)
- `L`: Left wheel velocity in m/s (negative = reverse)
- `R`: Right wheel velocity in m/s (negative = reverse)

### Motion Examples

```json
// Forward
{"T": 1, "L": 0.15, "R": 0.15}

// Reverse
{"T": 1, "L": -0.15, "R": -0.15}

// Spin clockwise (reliable)
{"T": 1, "L": 0.35, "R": -0.35}

// Spin counter-clockwise (reliable)
{"T": 1, "L": -0.35, "R": 0.35}

// Arc left (reliable)
{"T": 1, "L": 0.08, "R": 0.40}

// Arc right (reliable)
{"T": 1, "L": 0.40, "R": 0.08}

// Stop
{"T": 1, "L": 0, "R": 0}
```

## Heartbeat / Safety Timeout

The rover has a built-in safety feature:

- **Timeout:** ~3 seconds
- **Behavior:** If no motion command is received within the timeout, the rover automatically stops
- **Purpose:** Prevents runaway if communication is lost
- **Implication:** Continuous motion requires sending commands at regular intervals (recommended: 4 Hz / every 250ms)

## Surface Considerations

Motion thresholds may vary based on:

- **Surface type:** Carpet requires more power than hard floors
- **Battery level:** Lower battery may reduce available torque
- **Weight/payload:** Additional weight increases friction
- **Wheel condition:** Worn wheels may have different grip

## Status Commands & Response Formats

### Battery/Chassis Info (T:130 - CMD_BASE_FEEDBACK)

```json
{
    "T": 1001,
    "L": 0,           // Left wheel speed
    "R": 0,           // Right wheel speed
    "r": 0.58,        // Roll (degrees)
    "p": -0.71,       // Pitch (degrees)
    "y": -173.17,     // Yaw (degrees)
    "temp": 32.22,    // Temperature (°C)
    "v": 11.20        // Battery voltage (V)
}
```

**Voltage reference:**
- Full charge: ~12.6V
- Nominal: ~11.1V
- Low: ~10.0V (consider recharging)

### IMU Data (T:126 - CMD_GET_IMU_DATA)

```json
{
    "T": 1002,
    "r": 0.89,        // Roll (degrees)
    "p": -0.74,       // Pitch (degrees)
    "y": -173.05,     // Yaw (degrees)
    "ax": 11.89,      // Accelerometer X (mg)
    "ay": 14.07,      // Accelerometer Y (mg)
    "az": 983.41,     // Accelerometer Z (mg) ~= 1g when flat
    "gx": 0.12,       // Gyroscope X (°/s)
    "gy": 0.13,       // Gyroscope Y (°/s)
    "gz": 0.22,       // Gyroscope Z (°/s)
    "mx": 39,         // Magnetometer X
    "my": -7,         // Magnetometer Y
    "mz": -69,        // Magnetometer Z
    "temp": 32.22     // Temperature (°C)
}
```

### WiFi Info (T:405 - CMD_WIFI_INFO)

```json
{
    "ip": "192.168.4.115",
    "rssi": -41,              // Signal strength (dBm, closer to 0 = better)
    "wifi_mode_on_boot": 3    // 0=off, 1=AP, 2=STA, 3=AP+STA
}
```

**RSSI reference:**
- Excellent: -30 to -50 dBm
- Good: -50 to -60 dBm
- Fair: -60 to -70 dBm
- Weak: below -70 dBm

## Tested Configuration

- **Model:** Waveshare WAVE ROVER
- **Firmware:** Stock ESP32 firmware
- **Surface:** Indoor hard floor
- **Test date:** December 2024

## Recommended Test Speeds

For motion verification testing:

```python
# Conservative speeds for general motion
forward_speed = 0.15  # m/s
reverse_speed = 0.15  # m/s

# Higher speeds for turns (overcome friction)
spin_speed = 0.35     # m/s per wheel (opposite directions)

# Arc turn speeds (large differential)
arc_fast = 0.40       # m/s (outer wheel)
arc_slow = 0.08       # m/s (inner wheel)
```
