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
DEFAULT_AP_IP = "192.168.4.1"
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

    def __init__(self, ip: str, timeout: float = HTTP_TIMEOUT, logger: Optional[logging.Logger] = None):
        self.ip = ip
        self.base_url = f"http://{ip}"
        self.timeout = timeout
        self.logger = logger or logging.getLogger("wave_rover")
        self._session = requests.Session()

    def send_command(self, command: dict, retries: int = MAX_RETRIES) -> CommandResult:
        """
        Send a JSON command to the rover via HTTP GET.

        Commands are sent to: http://<ip>/js?json=<command>
        """
        json_str = json.dumps(command, separators=(',', ':'))
        url = f"{self.base_url}/js?json={quote(json_str)}"

        start_time = time.perf_counter()

        for attempt in range(retries):
            try:
                response = self._session.get(url, timeout=self.timeout)
                latency_ms = (time.perf_counter() - start_time) * 1000

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
            except requests.exceptions.ConnectionError as e:
                self.logger.warning(f"Connection error on attempt {attempt + 1}/{retries}: {e}")
            except Exception as e:
                self.logger.error(f"Unexpected error: {e}")
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
# Provisioning
# =============================================================================

def provision_sta(
    client: RoverClient,
    sta_ssid: str,
    sta_password: str,
    ap_ssid: str = DEFAULT_AP_SSID,
    ap_password: str = DEFAULT_AP_PASSWORD,
    logger: Optional[logging.Logger] = None
) -> bool:
    """
    Configure the rover to connect to a Wi-Fi network in STA mode.

    This should be run while your computer is connected to the rover's AP (UGV).
    After successful configuration, the rover will connect to the specified
    Wi-Fi network and its IP will be shown on the OLED "ST" line.

    Command: {"T":404,"ap_ssid":"...","ap_password":"...","sta_ssid":"...","sta_password":"..."}
    """
    logger = logger or logging.getLogger("wave_rover")

    command = {
        "T": 404,
        "ap_ssid": ap_ssid,
        "ap_password": ap_password,
        "sta_ssid": sta_ssid,
        "sta_password": sta_password
    }

    logger.info(f"Sending STA provisioning command...")
    logger.info(f"  AP SSID: {ap_ssid}")
    logger.info(f"  STA SSID: {sta_ssid}")

    result = client.send_command(command)

    if result.success:
        logger.info("Provisioning command sent successfully!")
        logger.info("")
        logger.info("=" * 60)
        logger.info("NEXT STEPS:")
        logger.info("=" * 60)
        logger.info("1. The rover will reboot and connect to your Wi-Fi network")
        logger.info("2. Check the rover's OLED display for the 'ST' line")
        logger.info("3. Note the IP address shown on the 'ST' line")
        logger.info("4. Connect your computer back to your normal Wi-Fi")
        logger.info("5. Run: python wave_rover_controller.py ping --ip <ST_IP>")
        logger.info("6. Run: python wave_rover_controller.py run-motion-suite --ip <ST_IP>")
        logger.info("=" * 60)
        return True
    else:
        logger.error(f"Provisioning failed: {result.error_message}")
        return False


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
# CLI Interface
# =============================================================================

def cmd_ping(args, logger):
    """Test connectivity to the rover."""
    client = RoverClient(args.ip, logger=logger)

    logger.info(f"Pinging rover at {args.ip}...")

    if client.ping():
        logger.info("SUCCESS - Rover is responding!")
        return 0
    else:
        logger.error("FAILED - Could not connect to rover")
        return 1


def cmd_provision_sta(args, logger):
    """Provision the rover to connect to a Wi-Fi network."""
    ip = args.ip or DEFAULT_AP_IP
    client = RoverClient(ip, logger=logger)

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
        logger=logger
    )

    return 0 if success else 1


def cmd_run_motion_suite(args, logger):
    """Run the full motion test suite."""
    client = RoverClient(args.ip, logger=logger)
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
    client = RoverClient(args.ip, logger=logger)
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
    client = RoverClient(args.ip, logger=logger)
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
"""
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose logging")

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Ping command
    ping_parser = subparsers.add_parser("ping", help="Test connectivity to rover")
    ping_parser.add_argument("--ip", required=True, help="Rover IP address")

    # Provision STA command
    provision_parser = subparsers.add_parser("provision-sta", help="Configure rover Wi-Fi (STA mode)")
    provision_parser.add_argument("--ip", help=f"Rover IP (default: {DEFAULT_AP_IP})")
    provision_parser.add_argument("--sta-ssid", required=True, help="Your Wi-Fi network SSID")
    provision_parser.add_argument("--sta-password", required=True, help="Your Wi-Fi network password")
    provision_parser.add_argument("--ap-ssid", default=DEFAULT_AP_SSID, help=f"Rover AP SSID (default: {DEFAULT_AP_SSID})")
    provision_parser.add_argument("--ap-password", default=DEFAULT_AP_PASSWORD, help=f"Rover AP password (default: {DEFAULT_AP_PASSWORD})")

    # Run motion suite command
    suite_parser = subparsers.add_parser("run-motion-suite", help="Run full motion test suite")
    suite_parser.add_argument("--ip", required=True, help="Rover IP address (from OLED 'ST' line)")
    suite_parser.add_argument("--speed", type=float, default=DEFAULT_TEST_SPEED, help=f"Test speed in m/s (default: {DEFAULT_TEST_SPEED})")
    suite_parser.add_argument("--duration", type=float, default=DEFAULT_MOTION_DURATION, help=f"Duration per motion in seconds (default: {DEFAULT_MOTION_DURATION})")
    suite_parser.add_argument("--settle-delay", type=float, default=DEFAULT_SETTLE_DELAY, help=f"Delay between motions (default: {DEFAULT_SETTLE_DELAY})")
    suite_parser.add_argument("--non-interactive", action="store_true", help="Run without user prompts")

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
        "ping": cmd_ping,
        "provision-sta": cmd_provision_sta,
        "run-motion-suite": cmd_run_motion_suite,
        "stop": cmd_stop,
        "move": cmd_move,
    }

    return commands[args.command](args, logger)


if __name__ == "__main__":
    sys.exit(main())
