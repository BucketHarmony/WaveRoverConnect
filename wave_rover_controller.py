#!/usr/bin/env python3
"""
Waveshare WAVE ROVER Wi-Fi Controller & Motion Test Suite

A Python-based controller that connects to the Waveshare WAVE ROVER over Wi-Fi
and runs a full motion verification suite using the documented HTTP JSON interface.

Requirements:
    - Python 3.10+
    - requests library (pip install requests)

Usage:
    # Provision rover to connect to your Wi-Fi (run while connected to UGV AP)
    python wave_rover_controller.py provision-sta --sta-ssid "YourWiFi" --sta-password "password"

    # Run motion test suite (run while on your normal network)
    python wave_rover_controller.py run-motion-suite --ip 192.168.1.100

    # Quick connectivity test
    python wave_rover_controller.py ping --ip 192.168.1.100

References:
    - https://www.waveshare.com/wiki/WAVE_ROVER
    - https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional
from urllib.parse import quote

import requests

# =============================================================================
# Configuration & Constants
# =============================================================================

# Default rover settings
DEFAULT_AP_IP = "192.168.4.115"
DEFAULT_AP_SSID = "UGV"
DEFAULT_AP_PASSWORD = "12345678"

# HTTP settings
HTTP_TIMEOUT = 2.0  # seconds
MAX_RETRIES = 3

# Motion control settings
KEEPALIVE_INTERVAL = 0.25  # seconds (send commands at 4 Hz, well under 3s timeout)
SAFETY_STOP_TIMEOUT = 3.0  # seconds - rover auto-stops if no command received

# Default test parameters
DEFAULT_TEST_SPEED = 0.15  # m/s - conservative default
DEFAULT_MOTION_DURATION = 2.0  # seconds per motion primitive
DEFAULT_SETTLE_DELAY = 1.0  # seconds between primitives


# =============================================================================
# Logging Setup
# =============================================================================

class ColorFormatter(logging.Formatter):
    """Custom formatter with colors for terminal output."""

    COLORS = {
        'DEBUG': '\033[36m',     # Cyan
        'INFO': '\033[32m',      # Green
        'WARNING': '\033[33m',   # Yellow
        'ERROR': '\033[31m',     # Red
        'CRITICAL': '\033[35m',  # Magenta
        'RESET': '\033[0m'
    }

    def format(self, record):
        color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        reset = self.COLORS['RESET']
        record.levelname = f"{color}{record.levelname}{reset}"
        return super().format(record)


def setup_logging(verbose: bool = False) -> logging.Logger:
    """Configure logging with timestamps and colors."""
    logger = logging.getLogger("wave_rover")
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)

    handler = logging.StreamHandler()
    handler.setFormatter(ColorFormatter(
        '%(asctime)s.%(msecs)03d | %(levelname)s | %(message)s',
        datefmt='%H:%M:%S'
    ))
    logger.addHandler(handler)

    return logger


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class CommandResult:
    """Result of sending a command to the rover."""
    success: bool
    command: dict
    status_code: Optional[int] = None
    response_body: Optional[str] = None
    error_message: Optional[str] = None
    latency_ms: float = 0.0


@dataclass
class TestResult:
    """Result of a motion test primitive."""
    name: str
    passed: bool
    description: str
    commands_sent: int = 0
    avg_loop_rate_hz: float = 0.0
    notes: str = ""


@dataclass
class MotionPrimitive:
    """Definition of a motion test primitive."""
    name: str
    description: str
    left_speed: float
    right_speed: float
    duration: float = DEFAULT_MOTION_DURATION


# =============================================================================
# Rover HTTP Client
# =============================================================================

class RoverClient:
    """HTTP client for communicating with the WAVE ROVER."""

    def __init__(self, ip: str, timeout: float = HTTP_TIMEOUT, logger: Optional[logging.Logger] = None, verbose: bool = False):
        self.ip = ip
        self.base_url = f"http://{ip}"
        self.timeout = timeout
        self.logger = logger or logging.getLogger("wave_rover")
        self.verbose = verbose
        self._session = requests.Session()

    def send_command(self, command: dict, retries: int = MAX_RETRIES) -> CommandResult:
        """
        Send a JSON command to the rover via HTTP GET.

        Commands are sent to: http://<ip>/js?json=<command>
        """
        json_str = json.dumps(command, separators=(',', ':'))
        url = f"{self.base_url}/js?json={quote(json_str)}"

        start_time = time.perf_counter()

        # Verbose logging - request details
        if self.verbose:
            self.logger.info(f">>> SEND: {json_str}")
            self.logger.debug(f">>> URL:  {url}")

        for attempt in range(retries):
            try:
                response = self._session.get(url, timeout=self.timeout)
                latency_ms = (time.perf_counter() - start_time) * 1000

                # Verbose logging - response details
                if self.verbose:
                    self.logger.info(f"<<< RECV: HTTP {response.status_code} ({latency_ms:.1f}ms)")
                    self.logger.info(f"<<< BODY: {response.text[:500] if response.text else '(empty)'}")
                else:
                    self.logger.debug(
                        f"Command: {json_str} -> HTTP {response.status_code} "
                        f"({latency_ms:.1f}ms)"
                    )

                return CommandResult(
                    success=response.status_code == 200,
                    command=command,
                    status_code=response.status_code,
                    response_body=response.text,
                    latency_ms=latency_ms
                )

            except requests.exceptions.Timeout:
                self.logger.warning(f"Timeout on attempt {attempt + 1}/{retries}")
                if self.verbose:
                    self.logger.warning(f"<<< TIMEOUT after {self.timeout}s")
            except requests.exceptions.ConnectionError as e:
                self.logger.warning(f"Connection error on attempt {attempt + 1}/{retries}: {e}")
                if self.verbose:
                    self.logger.warning(f"<<< CONNECTION ERROR: {e}")
            except Exception as e:
                self.logger.error(f"Unexpected error: {e}")
                if self.verbose:
                    self.logger.error(f"<<< EXCEPTION: {type(e).__name__}: {e}")
                return CommandResult(
                    success=False,
                    command=command,
                    error_message=str(e),
                    latency_ms=(time.perf_counter() - start_time) * 1000
                )

        return CommandResult(
            success=False,
            command=command,
            error_message=f"Failed after {retries} retries",
            latency_ms=(time.perf_counter() - start_time) * 1000
        )

    def ping(self) -> bool:
        """Test connectivity by sending a stop command."""
        result = self.send_command({"T": 1, "L": 0, "R": 0}, retries=1)
        return result.success

    def close(self):
        """Close the HTTP session."""
        self._session.close()


# =============================================================================
# Motion Controller with E-STOP and Keepalive
# =============================================================================

class MotionController:
    """
    Motion controller with keepalive loop and emergency stop functionality.

    The rover stops automatically if no command is received within ~3 seconds,
    so this controller sends commands at regular intervals to maintain motion.
    """

    def __init__(self, client: RoverClient, logger: Optional[logging.Logger] = None):
        self.client = client
        self.logger = logger or logging.getLogger("wave_rover")

        # Current motion state
        self._current_left = 0.0
        self._current_right = 0.0
        self._lock = threading.Lock()

        # Keepalive thread
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_running = False

        # Statistics
        self._commands_sent = 0
        self._loop_times: list[float] = []

        # E-STOP flag
        self._estop_triggered = False

    def _build_speed_command(self, left: float, right: float) -> dict:
        """Build a CMD_SPEED_CTRL command. T=1 for wheel velocity control."""
        return {"T": 1, "L": round(left, 3), "R": round(right, 3)}

    def e_stop(self) -> bool:
        """
        Emergency stop - immediately halt all motion.

        This is the single-command E-STOP that should always be available.
        """
        self._estop_triggered = True
        self.stop_keepalive()

        with self._lock:
            self._current_left = 0.0
            self._current_right = 0.0

        self.logger.warning("E-STOP TRIGGERED - Sending stop command")

        # Send stop command multiple times for reliability
        for _ in range(3):
            result = self.client.send_command(self._build_speed_command(0, 0))
            if result.success:
                self.logger.info("E-STOP successful - rover stopped")
                return True
            time.sleep(0.1)

        self.logger.error("E-STOP FAILED - Could not confirm rover stop!")
        return False

    def stop(self) -> CommandResult:
        """Send a stop command (non-emergency)."""
        with self._lock:
            self._current_left = 0.0
            self._current_right = 0.0
        return self.client.send_command(self._build_speed_command(0, 0))

    def set_speed(self, left: float, right: float) -> None:
        """Set target wheel speeds (will be sent by keepalive loop)."""
        with self._lock:
            self._current_left = left
            self._current_right = right

    def _keepalive_loop(self):
        """Background thread that continuously sends motion commands."""
        self.logger.debug(f"Keepalive loop started (interval: {KEEPALIVE_INTERVAL}s)")

        while self._keepalive_running and not self._estop_triggered:
            loop_start = time.perf_counter()

            with self._lock:
                left = self._current_left
                right = self._current_right

            result = self.client.send_command(self._build_speed_command(left, right))

            if not result.success:
                self.logger.error(f"Keepalive command failed: {result.error_message}")
                self.e_stop()
                break

            self._commands_sent += 1
            loop_time = time.perf_counter() - loop_start
            self._loop_times.append(loop_time)

            # Sleep for the remainder of the interval
            sleep_time = max(0, KEEPALIVE_INTERVAL - loop_time)
            time.sleep(sleep_time)

        self.logger.debug("Keepalive loop stopped")

    def start_keepalive(self):
        """Start the keepalive background thread."""
        if self._keepalive_thread and self._keepalive_thread.is_alive():
            return

        self._keepalive_running = True
        self._estop_triggered = False
        self._commands_sent = 0
        self._loop_times = []

        self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._keepalive_thread.start()

    def stop_keepalive(self):
        """Stop the keepalive background thread."""
        self._keepalive_running = False
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=1.0)
            self._keepalive_thread = None

    def get_statistics(self) -> tuple[int, float]:
        """Return (commands_sent, average_loop_rate_hz)."""
        if not self._loop_times:
            return self._commands_sent, 0.0
        avg_loop_time = sum(self._loop_times) / len(self._loop_times)
        avg_rate = 1.0 / avg_loop_time if avg_loop_time > 0 else 0.0
        return self._commands_sent, avg_rate

    def run_motion(self, left: float, right: float, duration: float) -> tuple[int, float]:
        """
        Run a motion for specified duration with keepalive.

        Returns (commands_sent, avg_loop_rate_hz).
        """
        self.set_speed(left, right)
        self.start_keepalive()

        time.sleep(duration)

        self.stop_keepalive()
        self.stop()

        return self.get_statistics()


# =============================================================================
# Network Scanner
# =============================================================================

def scan_for_rover(ip: str, timeout: float = 0.5) -> Optional[dict]:
    """
    Check if a rover exists at the given IP address.

    Returns rover info dict if found, None otherwise.
    """
    try:
        url = f"http://{ip}/js?json={quote(json.dumps({'T': 130}))}"
        response = requests.get(url, timeout=timeout)

        if response.status_code == 200:
            # Try to parse response as JSON to verify it's a rover
            try:
                data = json.loads(response.text)
                # Check if it looks like a valid rover response (has expected fields)
                if isinstance(data, dict) and 'T' in data:
                    return {
                        "ip": ip,
                        "response": data,
                        "voltage": data.get("v"),
                        "temperature": data.get("temp")
                    }
            except json.JSONDecodeError:
                pass
    except (requests.exceptions.Timeout, requests.exceptions.ConnectionError):
        pass
    except Exception:
        pass

    return None


def scan_network(
    subnet: str,
    timeout: float = 0.5,
    max_workers: int = 50,
    logger: Optional[logging.Logger] = None,
    progress_callback: Optional[Callable[[int, int, str], None]] = None
) -> list[dict]:
    """
    Scan a /24 subnet for WAVE ROVER devices.

    Args:
        subnet: Base subnet (e.g., "192.168.1" or "192.168.1.0")
        timeout: Connection timeout per IP in seconds
        max_workers: Number of concurrent scanner threads
        logger: Logger instance
        progress_callback: Called with (current, total, ip) for progress updates

    Returns:
        List of dicts with info about found rovers
    """
    import concurrent.futures

    logger = logger or logging.getLogger("wave_rover")

    # Parse subnet - handle both "192.168.1" and "192.168.1.0" formats
    subnet_base = subnet.rsplit('.', 1)[0] if subnet.count('.') == 3 else subnet

    # Generate all 256 IPs to scan
    ips_to_scan = [f"{subnet_base}.{i}" for i in range(256)]

    found_rovers = []
    scanned = 0

    def scan_ip(ip: str) -> Optional[dict]:
        nonlocal scanned
        result = scan_for_rover(ip, timeout)
        scanned += 1
        if progress_callback:
            progress_callback(scanned, 256, ip)
        return result

    # Use thread pool for concurrent scanning
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        # Submit all scan tasks
        future_to_ip = {executor.submit(scan_ip, ip): ip for ip in ips_to_scan}

        # Collect results as they complete
        for future in concurrent.futures.as_completed(future_to_ip):
            try:
                result = future.result()
                if result:
                    found_rovers.append(result)
                    logger.info(f"Found rover at {result['ip']}!")
            except Exception as e:
                logger.debug(f"Error scanning: {e}")

    return found_rovers


# =============================================================================
# Provisioning
# =============================================================================

def provision_sta(
    client: RoverClient,
    sta_ssid: str,
    sta_password: str,
    ap_ssid: str = DEFAULT_AP_SSID,
    ap_password: str = DEFAULT_AP_PASSWORD,
    logger: Optional[logging.Logger] = None,
    reboot: bool = True
) -> bool:
    """
    Configure the rover to connect to a Wi-Fi network in STA mode.

    This should be run while your computer is connected to the rover's AP (UGV).
    After successful configuration, the rover will connect to the specified
    Wi-Fi network and its IP will be shown on the OLED "ST" line.

    Command sequence:
    1. T:401 - Set WiFi mode to AP+STA (cmd=3)
    2. T:404 - Set AP and STA credentials
    3. T:600 - Reboot ESP32 (optional)
    """
    logger = logger or logging.getLogger("wave_rover")

    logger.info("=" * 60)
    logger.info("STEP 1: Set WiFi mode to AP+STA")
    logger.info("=" * 60)

    # T:401 - Set WiFi mode on boot: 0=off, 1=AP, 2=STA, 3=AP+STA
    wifi_mode_cmd = {"T": 401, "cmd": 3}
    result = client.send_command(wifi_mode_cmd)
    if not result.success:
        logger.error(f"Failed to set WiFi mode: {result.error_message}")
        return False
    logger.info(f"WiFi mode set to AP+STA (response: {result.response_body})")

    time.sleep(0.5)

    logger.info("")
    logger.info("=" * 60)
    logger.info("STEP 2: Configure WiFi credentials")
    logger.info("=" * 60)
    logger.info(f"  AP SSID: {ap_ssid}")
    logger.info(f"  STA SSID: {sta_ssid}")

    # T:404 - Set both AP and STA credentials
    credentials_cmd = {
        "T": 404,
        "ap_ssid": ap_ssid,
        "ap_password": ap_password,
        "sta_ssid": sta_ssid,
        "sta_password": sta_password
    }
    result = client.send_command(credentials_cmd)
    if not result.success:
        logger.error(f"Failed to set credentials: {result.error_message}")
        return False
    logger.info(f"Credentials configured (response: {result.response_body})")

    time.sleep(0.5)

    logger.info("")
    logger.info("=" * 60)
    logger.info("STEP 3: Query WiFi info to verify")
    logger.info("=" * 60)

    # T:405 - Get current WiFi info
    info_cmd = {"T": 405}
    result = client.send_command(info_cmd)
    if result.success:
        logger.info(f"WiFi info: {result.response_body}")
    else:
        logger.warning(f"Could not query WiFi info: {result.error_message}")

    if reboot:
        time.sleep(0.5)

        logger.info("")
        logger.info("=" * 60)
        logger.info("STEP 4: Rebooting ESP32")
        logger.info("=" * 60)

        # T:600 - Reboot ESP32
        reboot_cmd = {"T": 600}
        result = client.send_command(reboot_cmd, retries=1)
        # Note: Reboot may not return a response as the device restarts immediately
        logger.info("Reboot command sent")

    logger.info("")
    logger.info("=" * 60)
    logger.info("PROVISIONING COMPLETE - NEXT STEPS:")
    logger.info("=" * 60)
    logger.info("1. Wait for the rover to reboot (~10 seconds)")
    logger.info("2. Check the rover's OLED display for the 'ST' line")
    logger.info("3. Note the IP address shown on the 'ST' line")
    logger.info("4. Connect your computer back to your normal Wi-Fi")
    logger.info("5. Run: python wave_rover_controller.py ping --ip <ST_IP>")
    logger.info("6. Run: python wave_rover_controller.py run-motion-suite --ip <ST_IP>")
    logger.info("=" * 60)
    return True


def get_wifi_info(client: RoverClient, logger: Optional[logging.Logger] = None) -> Optional[str]:
    """Query current WiFi information from the rover (T:405)."""
    logger = logger or logging.getLogger("wave_rover")
    result = client.send_command({"T": 405})
    if result.success:
        return result.response_body
    logger.error(f"Failed to get WiFi info: {result.error_message}")
    return None


def set_wifi_mode(client: RoverClient, mode: int, logger: Optional[logging.Logger] = None) -> bool:
    """
    Set WiFi mode on boot (T:401).

    Modes:
        0 = WiFi off
        1 = AP mode only
        2 = STA mode only
        3 = AP+STA mode
    """
    logger = logger or logging.getLogger("wave_rover")
    result = client.send_command({"T": 401, "cmd": mode})
    if result.success:
        logger.info(f"WiFi mode set to {mode} (response: {result.response_body})")
        return True
    logger.error(f"Failed to set WiFi mode: {result.error_message}")
    return False


def reboot_rover(client: RoverClient, logger: Optional[logging.Logger] = None) -> bool:
    """Reboot the ESP32 (T:600)."""
    logger = logger or logging.getLogger("wave_rover")
    logger.info("Sending reboot command...")
    result = client.send_command({"T": 600}, retries=1)
    # Device may not respond as it reboots immediately
    logger.info("Reboot command sent (device may not respond)")
    return True


# =============================================================================
# Motion Test Suite
# =============================================================================

class MotionTestSuite:
    """Full range-of-motion test suite for the WAVE ROVER."""

    def __init__(
        self,
        controller: MotionController,
        speed: float = DEFAULT_TEST_SPEED,
        duration: float = DEFAULT_MOTION_DURATION,
        settle_delay: float = DEFAULT_SETTLE_DELAY,
        logger: Optional[logging.Logger] = None,
        interactive: bool = True
    ):
        self.controller = controller
        self.speed = speed
        self.duration = duration
        self.settle_delay = settle_delay
        self.logger = logger or logging.getLogger("wave_rover")
        self.interactive = interactive
        self.results: list[TestResult] = []

    def _define_primitives(self) -> list[MotionPrimitive]:
        """Define all motion primitives to test."""
        v = self.speed
        return [
            # Basic motions
            MotionPrimitive("stop", "Stop (L=0, R=0) - baseline", 0, 0, 1.0),
            MotionPrimitive("forward", f"Forward (L={v}, R={v})", v, v),
            MotionPrimitive("reverse", f"Reverse (L={-v}, R={-v})", -v, -v),

            # Turns in place
            MotionPrimitive("spin_cw", f"Spin clockwise (L={v}, R={-v})", v, -v),
            MotionPrimitive("spin_ccw", f"Spin counter-clockwise (L={-v}, R={v})", -v, v),

            # Arc turns (while moving forward)
            MotionPrimitive("arc_left", f"Arc left (L={v*0.5}, R={v})", v * 0.5, v),
            MotionPrimitive("arc_right", f"Arc right (L={v}, R={v*0.5})", v, v * 0.5),

            # Speed ramp test
            MotionPrimitive("speed_ramp", "Speed ramp (will vary speeds)", 0, 0),
        ]

    def _prompt_user(self, message: str) -> bool:
        """Prompt user for confirmation if in interactive mode."""
        if not self.interactive:
            return True

        print()
        print("-" * 60)
        print(f"NEXT: {message}")
        print("-" * 60)
        response = input("Press ENTER to continue, 's' to skip, 'q' to quit: ").strip().lower()

        if response == 'q':
            raise KeyboardInterrupt("User requested quit")
        return response != 's'

    def _confirm_result(self, primitive_name: str) -> bool:
        """Ask user to confirm if motion was correct."""
        if not self.interactive:
            return True

        response = input(f"Did '{primitive_name}' perform correctly? (y/n): ").strip().lower()
        return response in ('y', 'yes', '')

    def _run_primitive(self, primitive: MotionPrimitive) -> TestResult:
        """Run a single motion primitive test."""
        self.logger.info(f"Testing: {primitive.name} - {primitive.description}")

        if primitive.name == "speed_ramp":
            return self._run_speed_ramp_test()

        # Run the motion
        commands_sent, avg_rate = self.controller.run_motion(
            primitive.left_speed,
            primitive.right_speed,
            primitive.duration
        )

        self.logger.info(f"  Commands sent: {commands_sent}, Avg rate: {avg_rate:.1f} Hz")

        # Confirm result
        passed = self._confirm_result(primitive.name)

        return TestResult(
            name=primitive.name,
            passed=passed,
            description=primitive.description,
            commands_sent=commands_sent,
            avg_loop_rate_hz=avg_rate,
            notes="User confirmed" if passed else "User reported failure"
        )

    def _run_speed_ramp_test(self) -> TestResult:
        """Test gradual speed increase and decrease."""
        speeds = [0.1, 0.15, 0.2, 0.25, 0.2, 0.15, 0.1]
        step_duration = 1.0

        self.logger.info("  Running speed ramp: " + " -> ".join(f"{s}" for s in speeds))

        total_commands = 0

        self.controller.start_keepalive()

        try:
            for speed in speeds:
                self.logger.info(f"    Speed: {speed} m/s")
                self.controller.set_speed(speed, speed)
                time.sleep(step_duration)
        finally:
            self.controller.stop_keepalive()
            self.controller.stop()

        commands_sent, avg_rate = self.controller.get_statistics()
        passed = self._confirm_result("speed_ramp")

        return TestResult(
            name="speed_ramp",
            passed=passed,
            description="Gradual speed increase/decrease",
            commands_sent=commands_sent,
            avg_loop_rate_hz=avg_rate,
            notes=f"Tested speeds: {speeds}"
        )

    def run_heartbeat_validation(self) -> TestResult:
        """
        Validate that rover stops automatically when commands cease.

        This tests the ~3 second safety timeout.
        """
        self.logger.info("=" * 60)
        self.logger.info("HEARTBEAT VALIDATION TEST")
        self.logger.info("=" * 60)
        self.logger.info("This test will:")
        self.logger.info("1. Start the rover moving forward")
        self.logger.info("2. STOP sending commands (but not send stop)")
        self.logger.info("3. Wait and observe - rover should auto-stop within ~3 seconds")

        if not self._prompt_user("Heartbeat validation - rover will auto-stop"):
            return TestResult(
                name="heartbeat_validation",
                passed=False,
                description="Safety auto-stop test",
                notes="Skipped by user"
            )

        # Start motion (short burst, then stop sending)
        self.logger.info("Starting motion...")
        self.controller.set_speed(self.speed, self.speed)
        self.controller.start_keepalive()
        time.sleep(1.0)  # Move for 1 second

        self.logger.info("STOPPING command transmission (NOT sending stop command)...")
        self.controller.stop_keepalive()
        # NOTE: We intentionally do NOT call self.controller.stop() here

        self.logger.info("Observe the rover - it should auto-stop within ~3 seconds...")
        self.logger.info("Waiting 5 seconds...")
        time.sleep(5.0)

        # Now send stop to be safe
        self.controller.stop()

        passed = self._confirm_result("heartbeat_validation (did rover auto-stop?)")

        return TestResult(
            name="heartbeat_validation",
            passed=passed,
            description="Safety auto-stop when commands cease",
            notes="Rover should stop within ~3 seconds of last command"
        )

    def run(self) -> list[TestResult]:
        """Run the complete motion test suite."""
        self.results = []
        primitives = self._define_primitives()

        self.logger.info("=" * 60)
        self.logger.info("WAVE ROVER MOTION TEST SUITE")
        self.logger.info("=" * 60)
        self.logger.info(f"Test speed: {self.speed} m/s")
        self.logger.info(f"Duration per primitive: {self.duration} seconds")
        self.logger.info(f"Settle delay: {self.settle_delay} seconds")
        self.logger.info(f"Total primitives: {len(primitives)}")
        self.logger.info("=" * 60)

        try:
            for primitive in primitives:
                if not self._prompt_user(primitive.description):
                    self.results.append(TestResult(
                        name=primitive.name,
                        passed=False,
                        description=primitive.description,
                        notes="Skipped by user"
                    ))
                    continue

                result = self._run_primitive(primitive)
                self.results.append(result)

                # Settle delay between primitives
                self.logger.info(f"  Settling for {self.settle_delay}s...")
                time.sleep(self.settle_delay)

            # Heartbeat validation test
            heartbeat_result = self.run_heartbeat_validation()
            self.results.append(heartbeat_result)

        except KeyboardInterrupt:
            self.logger.warning("Test suite interrupted by user")
            raise

        return self.results

    def print_summary(self):
        """Print a summary of test results."""
        print()
        print("=" * 60)
        print("TEST RESULTS SUMMARY")
        print("=" * 60)

        passed = sum(1 for r in self.results if r.passed)
        failed = sum(1 for r in self.results if not r.passed)

        for result in self.results:
            status = "\033[32mPASS\033[0m" if result.passed else "\033[31mFAIL\033[0m"
            print(f"  [{status}] {result.name}: {result.description}")
            if result.commands_sent > 0:
                print(f"         Commands: {result.commands_sent}, Rate: {result.avg_loop_rate_hz:.1f} Hz")
            if result.notes:
                print(f"         Notes: {result.notes}")

        print()
        print("-" * 60)
        print(f"TOTAL: {passed} passed, {failed} failed out of {len(self.results)} tests")

        if failed == 0:
            print("\033[32mALL TESTS PASSED!\033[0m")
        else:
            print("\033[31mSOME TESTS FAILED\033[0m")

        print("=" * 60)


# =============================================================================
# Automated Test Suite (No User Input)
# =============================================================================

def run_automated_test(
    controller: MotionController,
    speed: float = DEFAULT_TEST_SPEED,
    duration: float = DEFAULT_MOTION_DURATION,
    settle_delay: float = DEFAULT_SETTLE_DELAY,
    logger: Optional[logging.Logger] = None,
    skip_heartbeat: bool = False
) -> list[dict]:
    """
    Run a fully automated motion test suite with no user input required.

    Displays all commands and status in real-time to the console.
    Returns a list of test results.
    """
    logger = logger or logging.getLogger("wave_rover")
    results = []

    # Define test sequence
    # Spin turns need decent power but not too much
    spin_speed = 0.35

    # Arc turns need big speed differential and higher power to overcome friction
    arc_fast = 0.4   # Fast wheel
    arc_slow = 0.08  # Slow wheel (almost stopped to force arc)

    tests = [
        {"name": "STOP", "left": 0, "right": 0, "duration": 1.0,
         "desc": "Baseline stop - wheels should not move"},
        {"name": "FORWARD", "left": speed, "right": speed, "duration": duration,
         "desc": "Both wheels forward at same speed"},
        {"name": "REVERSE", "left": -speed, "right": -speed, "duration": duration,
         "desc": "Both wheels backward at same speed"},
        {"name": "SPIN CLOCKWISE", "left": spin_speed, "right": -spin_speed, "duration": duration,
         "desc": f"Rotate in place clockwise ({spin_speed} m/s)"},
        {"name": "SPIN COUNTER-CW", "left": -spin_speed, "right": spin_speed, "duration": duration,
         "desc": f"Rotate in place counter-clockwise ({spin_speed} m/s)"},
        {"name": "ARC LEFT", "left": arc_slow, "right": arc_fast, "duration": duration,
         "desc": f"Curve left (L={arc_slow}, R={arc_fast} m/s)"},
        {"name": "ARC RIGHT", "left": arc_fast, "right": arc_slow, "duration": duration,
         "desc": f"Curve right (L={arc_fast}, R={arc_slow} m/s)"},
        {"name": "STOP", "left": 0, "right": 0, "duration": 1.0,
         "desc": "Final stop"},
    ]

    # Print header
    print()
    print("=" * 70)
    print("  AUTOMATED MOTION TEST SUITE - NO INPUT REQUIRED")
    print("=" * 70)
    print(f"  Speed: {speed} m/s | Duration: {duration}s | Settle: {settle_delay}s")
    print(f"  Tests: {len(tests)} motion primitives" + (" + heartbeat test" if not skip_heartbeat else ""))
    print("=" * 70)
    print()
    print("  Press Ctrl+C at any time to emergency stop")
    print()
    time.sleep(2)  # Give user time to read

    try:
        for i, test in enumerate(tests, 1):
            # Announce test
            print()
            print("-" * 70)
            print(f"  [{i}/{len(tests)}] {test['name']}")
            print(f"  {test['desc']}")
            print(f"  Command: L={test['left']:.3f} m/s, R={test['right']:.3f} m/s, Duration={test['duration']}s")
            print("-" * 70)

            # Countdown
            print("  Starting in: ", end="", flush=True)
            for countdown in [3, 2, 1]:
                print(f"{countdown}...", end="", flush=True)
                time.sleep(0.5)
            print(" GO!")

            # Execute motion
            start_time = time.time()
            commands_sent, avg_rate = controller.run_motion(
                test["left"], test["right"], test["duration"]
            )
            elapsed = time.time() - start_time

            # Report results
            print(f"  ✓ Complete | Sent {commands_sent} commands | Rate: {avg_rate:.1f} Hz | Time: {elapsed:.2f}s")

            results.append({
                "name": test["name"],
                "left": test["left"],
                "right": test["right"],
                "duration": test["duration"],
                "commands_sent": commands_sent,
                "avg_rate_hz": avg_rate,
                "elapsed": elapsed,
                "success": True
            })

            # Settle delay
            if settle_delay > 0 and i < len(tests):
                print(f"  Settling for {settle_delay}s...")
                time.sleep(settle_delay)

        # Speed ramp test
        print()
        print("-" * 70)
        print(f"  [RAMP] SPEED RAMP TEST")
        print(f"  Gradually increasing then decreasing speed")
        print("-" * 70)

        ramp_speeds = [0.05, 0.10, 0.15, 0.20, 0.25, 0.20, 0.15, 0.10, 0.05]
        step_duration = 0.8

        print("  Starting in: ", end="", flush=True)
        for countdown in [3, 2, 1]:
            print(f"{countdown}...", end="", flush=True)
            time.sleep(0.5)
        print(" GO!")

        controller.start_keepalive()
        ramp_start = time.time()

        try:
            for spd in ramp_speeds:
                print(f"  → Speed: {spd:.2f} m/s")
                controller.set_speed(spd, spd)
                time.sleep(step_duration)
        finally:
            controller.stop_keepalive()
            controller.stop()

        ramp_elapsed = time.time() - ramp_start
        ramp_cmds, ramp_rate = controller.get_statistics()
        print(f"  ✓ Ramp complete | Sent {ramp_cmds} commands | Rate: {ramp_rate:.1f} Hz | Time: {ramp_elapsed:.2f}s")

        results.append({
            "name": "SPEED_RAMP",
            "speeds": ramp_speeds,
            "commands_sent": ramp_cmds,
            "avg_rate_hz": ramp_rate,
            "elapsed": ramp_elapsed,
            "success": True
        })

        # Heartbeat validation test
        if not skip_heartbeat:
            print()
            print("-" * 70)
            print(f"  [SAFETY] HEARTBEAT VALIDATION TEST")
            print(f"  Will start moving, then STOP sending commands")
            print(f"  Rover should auto-stop within ~3 seconds")
            print("-" * 70)

            print("  Starting in: ", end="", flush=True)
            for countdown in [3, 2, 1]:
                print(f"{countdown}...", end="", flush=True)
                time.sleep(0.5)
            print(" GO!")

            # Start moving
            print(f"  → Moving forward at {speed} m/s...")
            controller.set_speed(speed, speed)
            controller.start_keepalive()
            time.sleep(1.5)

            # Stop sending commands (but don't send stop)
            print(f"  → STOPPING command transmission (NOT sending stop)...")
            controller.stop_keepalive()

            # Wait and observe
            print(f"  → Waiting for auto-stop (should happen within ~3s)...")
            for sec in range(5):
                time.sleep(1)
                print(f"    {sec + 1}s elapsed...")

            # Now send explicit stop
            controller.stop()
            print(f"  ✓ Heartbeat test complete - check if rover stopped automatically")

            results.append({
                "name": "HEARTBEAT_VALIDATION",
                "success": True,
                "note": "Manual verification required"
            })

        # Final stop
        controller.stop()

    except KeyboardInterrupt:
        print()
        print("  !!! INTERRUPTED - Emergency stopping !!!")
        controller.e_stop()
        raise
    except Exception as e:
        print(f"  !!! ERROR: {e} !!!")
        controller.e_stop()
        raise

    # Print summary
    print()
    print("=" * 70)
    print("  TEST SUMMARY")
    print("=" * 70)

    total_commands = sum(r.get("commands_sent", 0) for r in results)
    total_time = sum(r.get("elapsed", 0) for r in results)

    for r in results:
        status = "✓" if r.get("success") else "✗"
        if "commands_sent" in r:
            print(f"  {status} {r['name']}: {r['commands_sent']} cmds, {r.get('avg_rate_hz', 0):.1f} Hz")
        else:
            print(f"  {status} {r['name']}: {r.get('note', 'OK')}")

    print("-" * 70)
    print(f"  Total: {len(results)} tests | {total_commands} commands | {total_time:.1f}s runtime")
    print("=" * 70)
    print()

    return results


# =============================================================================
# CLI Interface
# =============================================================================

def cmd_auto_test(args, logger):
    """Run the fully automated motion test suite."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)
    controller = MotionController(client, logger=logger)

    # Set up signal handler for Ctrl+C
    def signal_handler(sig, frame):
        print("\n  !!! Ctrl+C - Emergency stopping !!!")
        controller.e_stop()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    # Test connectivity first
    logger.info(f"Testing connectivity to {args.ip}...")
    if not client.ping():
        logger.error("Cannot connect to rover. Check IP address and network connection.")
        return 1
    logger.info("Connection successful!")

    try:
        run_automated_test(
            controller,
            speed=args.speed,
            duration=args.duration,
            settle_delay=args.settle_delay,
            logger=logger,
            skip_heartbeat=args.skip_heartbeat
        )
    except KeyboardInterrupt:
        logger.warning("Test interrupted")
    except Exception as e:
        logger.error(f"Error during test: {e}")
        controller.e_stop()
        raise
    finally:
        controller.stop()
        client.close()

    return 0


def cmd_scan(args, logger):
    """Scan a subnet for WAVE ROVER devices."""
    subnet = args.subnet
    timeout = args.timeout
    workers = args.workers

    print()
    print("=" * 60)
    print("  WAVE ROVER NETWORK SCANNER")
    print("=" * 60)
    print(f"  Subnet: {subnet}.0/24 (256 addresses)")
    print(f"  Timeout: {timeout}s per IP")
    print(f"  Workers: {workers} concurrent connections")
    print("=" * 60)
    print()

    # Progress tracking
    last_percent = -1

    def progress_callback(current: int, total: int, ip: str):
        nonlocal last_percent
        percent = (current * 100) // total
        if percent != last_percent and percent % 10 == 0:
            print(f"  Scanning... {percent}% ({current}/{total})")
            last_percent = percent

    print("  Starting scan...")
    start_time = time.time()

    rovers = scan_network(
        subnet=subnet,
        timeout=timeout,
        max_workers=workers,
        logger=logger,
        progress_callback=progress_callback
    )

    elapsed = time.time() - start_time

    print()
    print("=" * 60)
    print("  SCAN RESULTS")
    print("=" * 60)
    print(f"  Scanned 256 addresses in {elapsed:.1f}s")
    print()

    if rovers:
        print(f"  Found {len(rovers)} rover(s):")
        print()
        for rover in rovers:
            print(f"    IP: {rover['ip']}")
            if rover.get('voltage'):
                print(f"        Voltage: {rover['voltage']}V")
            if rover.get('temperature'):
                print(f"        Temperature: {rover['temperature']}°C")
            print()
    else:
        print("  No rovers found on this subnet.")
        print()
        print("  Tips:")
        print("    - Make sure the rover is powered on")
        print("    - Check if you're on the same network as the rover")
        print("    - Try scanning the rover's AP subnet: 192.168.4")
        print()

    print("=" * 60)
    print()

    return 0 if rovers else 1


def cmd_ping(args, logger):
    """Test connectivity to the rover."""
    client = RoverClient(args.ip, logger=logger, verbose=getattr(args, 'verbose', False))

    logger.info(f"Pinging rover at {args.ip}...")

    if client.ping():
        logger.info("SUCCESS - Rover is responding!")
        return 0
    else:
        logger.error("FAILED - Could not connect to rover")
        return 1


def cmd_status(args, logger):
    """Get battery level and status information from the rover."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    print()
    print("=" * 60)
    print("  WAVE ROVER STATUS")
    print("=" * 60)

    # Status commands to query
    status_commands = [
        {"T": 130, "name": "Chassis/Battery Info", "desc": "CMD_BASE_FEEDBACK"},
        {"T": 126, "name": "IMU Data", "desc": "CMD_GET_IMU_DATA"},
        {"T": 405, "name": "WiFi Info", "desc": "CMD_WIFI_INFO"},
        {"T": 302, "name": "MAC Address", "desc": "CMD_GET_MAC_ADDRESS"},
        {"T": 601, "name": "Flash Space", "desc": "CMD_FREE_FLASH_SPACE"},
    ]

    results = {}

    for cmd_info in status_commands:
        cmd = {"T": cmd_info["T"]}
        print()
        print(f"  [{cmd_info['name']}] ({cmd_info['desc']})")
        print(f"  Request: {json.dumps(cmd)}")

        result = client.send_command(cmd, retries=1)

        if result.success:
            response = result.response_body
            results[cmd_info["name"]] = response

            # Try to parse as JSON for pretty printing
            try:
                data = json.loads(response)
                print(f"  Response: {json.dumps(data, indent=4)}")

                # Extract and highlight key values if present
                if isinstance(data, dict):
                    # Look for common battery/voltage fields
                    for key in ['v', 'V', 'voltage', 'Voltage', 'bat', 'battery', 'Battery']:
                        if key in data:
                            print(f"  >>> VOLTAGE: {data[key]}")
                    # Look for IMU data
                    for key in ['r', 'p', 'y', 'roll', 'pitch', 'yaw']:
                        if key in data:
                            print(f"  >>> {key.upper()}: {data[key]}")

            except json.JSONDecodeError:
                # Not JSON, print raw (truncated if long)
                if len(response) > 200:
                    print(f"  Response: {response[:200]}...")
                else:
                    print(f"  Response: {response}")
        else:
            print(f"  ERROR: {result.error_message}")
            results[cmd_info["name"]] = None

    print()
    print("=" * 60)
    print()

    client.close()
    return 0


def cmd_battery(args, logger):
    """Get just the battery/chassis info (quick check)."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    # CMD_BASE_FEEDBACK - gets chassis info including voltage
    result = client.send_command({"T": 130}, retries=2)

    if result.success:
        print()
        print("Battery/Chassis Info:")
        print("-" * 40)

        try:
            data = json.loads(result.response_body)
            print(json.dumps(data, indent=2))

            # Try to extract voltage
            if isinstance(data, dict):
                for key in ['v', 'V', 'voltage', 'Voltage', 'bat', 'battery']:
                    if key in data:
                        print()
                        print(f"  VOLTAGE: {data[key]} V")
                        break
        except json.JSONDecodeError:
            print(result.response_body)

        print()
        return 0
    else:
        logger.error(f"Failed to get battery info: {result.error_message}")
        return 1


def cmd_imu(args, logger):
    """Get IMU sensor data."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    # CMD_GET_IMU_DATA
    result = client.send_command({"T": 126}, retries=2)

    if result.success:
        print()
        print("IMU Data:")
        print("-" * 40)

        try:
            data = json.loads(result.response_body)
            print(json.dumps(data, indent=2))
        except json.JSONDecodeError:
            print(result.response_body)

        print()
        return 0
    else:
        logger.error(f"Failed to get IMU data: {result.error_message}")
        return 1


def cmd_provision_sta(args, logger):
    """Provision the rover to connect to a Wi-Fi network."""
    ip = args.ip or DEFAULT_AP_IP
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(ip, logger=logger, verbose=verbose)

    logger.info("=" * 60)
    logger.info("WAVE ROVER STA PROVISIONING")
    logger.info("=" * 60)
    logger.info(f"Connecting to rover at {ip}")
    logger.info("(You should be connected to the UGV access point)")
    logger.info("")

    success = provision_sta(
        client,
        sta_ssid=args.sta_ssid,
        sta_password=args.sta_password,
        ap_ssid=args.ap_ssid,
        ap_password=args.ap_password,
        logger=logger,
        reboot=not getattr(args, 'no_reboot', False)
    )

    return 0 if success else 1


def cmd_wifi_info(args, logger):
    """Query WiFi information from the rover."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    logger.info(f"Querying WiFi info from {args.ip}...")
    info = get_wifi_info(client, logger)

    if info:
        logger.info(f"WiFi Info: {info}")
        return 0
    return 1


def cmd_wifi_mode(args, logger):
    """Set WiFi mode on boot."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    mode_names = {0: "OFF", 1: "AP only", 2: "STA only", 3: "AP+STA"}
    logger.info(f"Setting WiFi mode to {args.mode} ({mode_names.get(args.mode, 'unknown')})...")

    if set_wifi_mode(client, args.mode, logger):
        return 0
    return 1


def cmd_reboot(args, logger):
    """Reboot the ESP32."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    logger.info(f"Rebooting rover at {args.ip}...")
    reboot_rover(client, logger)
    logger.info("Reboot command sent. Rover should restart in a few seconds.")
    return 0


def cmd_send(args, logger):
    """Send a raw JSON command to the rover."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)

    try:
        command = json.loads(args.json)
    except json.JSONDecodeError as e:
        logger.error(f"Invalid JSON: {e}")
        return 1

    logger.info(f"Sending command: {args.json}")
    result = client.send_command(command)

    logger.info(f"Status: HTTP {result.status_code}")
    logger.info(f"Response: {result.response_body}")

    return 0 if result.success else 1


def cmd_run_motion_suite(args, logger):
    """Run the full motion test suite."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)
    controller = MotionController(client, logger=logger)

    # Set up signal handler for Ctrl+C
    def signal_handler(sig, frame):
        logger.warning("Caught Ctrl+C - Emergency stopping!")
        controller.e_stop()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    # Test connectivity first
    logger.info(f"Testing connectivity to {args.ip}...")
    if not client.ping():
        logger.error("Cannot connect to rover. Check IP address and network connection.")
        return 1
    logger.info("Connection successful!")

    suite = MotionTestSuite(
        controller,
        speed=args.speed,
        duration=args.duration,
        settle_delay=args.settle_delay,
        logger=logger,
        interactive=not args.non_interactive
    )

    try:
        suite.run()
    except KeyboardInterrupt:
        logger.warning("Test interrupted - stopping rover")
        controller.e_stop()
    except Exception as e:
        logger.error(f"Error during test: {e}")
        controller.e_stop()
        raise
    finally:
        # Always ensure rover is stopped
        controller.stop()
        client.close()

    suite.print_summary()

    failed = sum(1 for r in suite.results if not r.passed)
    return 1 if failed > 0 else 0


def cmd_stop(args, logger):
    """Send an emergency stop command."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)
    controller = MotionController(client, logger=logger)

    logger.info(f"Sending E-STOP to {args.ip}...")

    if controller.e_stop():
        logger.info("Rover stopped successfully")
        return 0
    else:
        logger.error("Failed to stop rover!")
        return 1


def cmd_move(args, logger):
    """Send a single motion command (for testing)."""
    verbose = getattr(args, 'verbose', False)
    client = RoverClient(args.ip, logger=logger, verbose=verbose)
    controller = MotionController(client, logger=logger)

    def signal_handler(sig, frame):
        logger.warning("Caught Ctrl+C - Stopping!")
        controller.e_stop()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    logger.info(f"Moving: L={args.left}, R={args.right} for {args.duration}s")
    logger.info("Press Ctrl+C to stop at any time")

    try:
        controller.run_motion(args.left, args.right, args.duration)
        logger.info("Motion complete")
    finally:
        controller.stop()
        client.close()

    return 0


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Waveshare WAVE ROVER Wi-Fi Controller & Motion Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Scan for rovers on your network
  %(prog)s scan --subnet 192.168.1

  # Scan the rover's AP network (default)
  %(prog)s scan

  # Provision rover to your Wi-Fi (run while on UGV network)
  %(prog)s provision-sta --sta-ssid "MyWiFi" --sta-password "mypassword"

  # Test connectivity
  %(prog)s ping --ip 192.168.1.100

  # Run full motion test suite
  %(prog)s run-motion-suite --ip 192.168.1.100

  # Run with faster speed
  %(prog)s run-motion-suite --ip 192.168.1.100 --speed 0.25

  # Emergency stop
  %(prog)s stop --ip 192.168.1.100

  # Manual move command
  %(prog)s move --ip 192.168.1.100 --left 0.1 --right 0.1 --duration 2

  # Debug commands (with verbose logging)
  %(prog)s -v wifi-info --ip 192.168.4.1
  %(prog)s -v send --ip 192.168.4.1 --json '{"T":405}'
  %(prog)s reboot --ip 192.168.4.1
"""
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose logging (show all sent/received data)")

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Scan command
    scan_parser = subparsers.add_parser("scan", help="Scan subnet to find rovers")
    scan_parser.add_argument("--subnet", default="192.168.4",
                             help="Subnet to scan (e.g., 192.168.1 or 192.168.4). Default: 192.168.4")
    scan_parser.add_argument("--timeout", type=float, default=0.5,
                             help="Timeout per IP in seconds (default: 0.5)")
    scan_parser.add_argument("--workers", type=int, default=50,
                             help="Number of concurrent connections (default: 50)")

    # Ping command
    ping_parser = subparsers.add_parser("ping", help="Test connectivity to rover")
    ping_parser.add_argument("--ip", required=True, help="Rover IP address")

    # Status command (all info)
    status_parser = subparsers.add_parser("status", help="Get all status info (battery, IMU, WiFi, etc.)")
    status_parser.add_argument("--ip", default=DEFAULT_AP_IP, help=f"Rover IP address (default: {DEFAULT_AP_IP})")

    # Battery command (quick)
    battery_parser = subparsers.add_parser("battery", help="Get battery/chassis info")
    battery_parser.add_argument("--ip", default=DEFAULT_AP_IP, help=f"Rover IP address (default: {DEFAULT_AP_IP})")

    # IMU command
    imu_parser = subparsers.add_parser("imu", help="Get IMU sensor data")
    imu_parser.add_argument("--ip", default=DEFAULT_AP_IP, help=f"Rover IP address (default: {DEFAULT_AP_IP})")

    # Provision STA command
    provision_parser = subparsers.add_parser("provision-sta", help="Configure rover Wi-Fi (STA mode)")
    provision_parser.add_argument("--ip", help=f"Rover IP (default: {DEFAULT_AP_IP})")
    provision_parser.add_argument("--sta-ssid", required=True, help="Your Wi-Fi network SSID")
    provision_parser.add_argument("--sta-password", required=True, help="Your Wi-Fi network password")
    provision_parser.add_argument("--ap-ssid", default=DEFAULT_AP_SSID, help=f"Rover AP SSID (default: {DEFAULT_AP_SSID})")
    provision_parser.add_argument("--ap-password", default=DEFAULT_AP_PASSWORD, help=f"Rover AP password (default: {DEFAULT_AP_PASSWORD})")
    provision_parser.add_argument("--no-reboot", action="store_true", help="Don't reboot after provisioning")

    # WiFi info command
    wifi_info_parser = subparsers.add_parser("wifi-info", help="Query current WiFi information")
    wifi_info_parser.add_argument("--ip", required=True, help="Rover IP address")

    # WiFi mode command
    wifi_mode_parser = subparsers.add_parser("wifi-mode", help="Set WiFi mode on boot")
    wifi_mode_parser.add_argument("--ip", required=True, help="Rover IP address")
    wifi_mode_parser.add_argument("--mode", type=int, required=True, choices=[0, 1, 2, 3],
                                   help="WiFi mode: 0=off, 1=AP, 2=STA, 3=AP+STA")

    # Reboot command
    reboot_parser = subparsers.add_parser("reboot", help="Reboot the ESP32")
    reboot_parser.add_argument("--ip", required=True, help="Rover IP address")

    # Send raw command
    send_parser = subparsers.add_parser("send", help="Send a raw JSON command")
    send_parser.add_argument("--ip", required=True, help="Rover IP address")
    send_parser.add_argument("--json", required=True, help='JSON command (e.g., \'{"T":405}\')')

    # Run motion suite command (interactive)
    suite_parser = subparsers.add_parser("run-motion-suite", help="Run full motion test suite (interactive)")
    suite_parser.add_argument("--ip", required=True, help="Rover IP address (from OLED 'ST' line)")
    suite_parser.add_argument("--speed", type=float, default=DEFAULT_TEST_SPEED, help=f"Test speed in m/s (default: {DEFAULT_TEST_SPEED})")
    suite_parser.add_argument("--duration", type=float, default=DEFAULT_MOTION_DURATION, help=f"Duration per motion in seconds (default: {DEFAULT_MOTION_DURATION})")
    suite_parser.add_argument("--settle-delay", type=float, default=DEFAULT_SETTLE_DELAY, help=f"Delay between motions (default: {DEFAULT_SETTLE_DELAY})")
    suite_parser.add_argument("--non-interactive", action="store_true", help="Run without user prompts")

    # Automated test suite (no user input)
    auto_parser = subparsers.add_parser("auto-test", help="Run automated motion test (no user input)")
    auto_parser.add_argument("--ip", default=DEFAULT_AP_IP, help=f"Rover IP address (default: {DEFAULT_AP_IP})")
    auto_parser.add_argument("--speed", type=float, default=DEFAULT_TEST_SPEED, help=f"Test speed in m/s (default: {DEFAULT_TEST_SPEED})")
    auto_parser.add_argument("--duration", type=float, default=DEFAULT_MOTION_DURATION, help=f"Duration per motion in seconds (default: {DEFAULT_MOTION_DURATION})")
    auto_parser.add_argument("--settle-delay", type=float, default=DEFAULT_SETTLE_DELAY, help=f"Delay between motions (default: {DEFAULT_SETTLE_DELAY})")
    auto_parser.add_argument("--skip-heartbeat", action="store_true", help="Skip the heartbeat validation test")

    # Stop command
    stop_parser = subparsers.add_parser("stop", help="Emergency stop")
    stop_parser.add_argument("--ip", required=True, help="Rover IP address")

    # Move command
    move_parser = subparsers.add_parser("move", help="Send a motion command")
    move_parser.add_argument("--ip", required=True, help="Rover IP address")
    move_parser.add_argument("--left", "-l", type=float, required=True, help="Left wheel speed (m/s)")
    move_parser.add_argument("--right", "-r", type=float, required=True, help="Right wheel speed (m/s)")
    move_parser.add_argument("--duration", "-d", type=float, default=2.0, help="Duration in seconds (default: 2)")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    logger = setup_logging(args.verbose)

    # Dispatch to command handler
    commands = {
        "scan": cmd_scan,
        "ping": cmd_ping,
        "status": cmd_status,
        "battery": cmd_battery,
        "imu": cmd_imu,
        "provision-sta": cmd_provision_sta,
        "wifi-info": cmd_wifi_info,
        "wifi-mode": cmd_wifi_mode,
        "reboot": cmd_reboot,
        "send": cmd_send,
        "run-motion-suite": cmd_run_motion_suite,
        "auto-test": cmd_auto_test,
        "stop": cmd_stop,
        "move": cmd_move,
    }

    return commands[args.command](args, logger)


if __name__ == "__main__":
    sys.exit(main())
