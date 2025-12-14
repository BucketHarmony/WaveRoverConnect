# Test Rover Connection

Run connectivity test and motion suite on the WAVE ROVER.

## Instructions

1. First, ask the user for the rover's IP address (from the OLED "ST" line)
2. Run `python wave_rover_controller.py ping --ip <IP>` to test connectivity
3. If successful, ask if they want to run the full motion test suite
4. If yes, run `python wave_rover_controller.py run-motion-suite --ip <IP>`
5. Report the results
