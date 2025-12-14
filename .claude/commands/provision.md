# Provision Rover Wi-Fi

Help the user provision their WAVE ROVER to connect to their Wi-Fi network.

## Instructions

1. Remind the user they need to be connected to the rover's AP:
   - SSID: `UGV`
   - Password: `12345678`

2. Ask for their home Wi-Fi credentials:
   - SSID (network name)
   - Password

3. Run the provisioning command:
   ```
   python wave_rover_controller.py provision-sta --sta-ssid "<SSID>" --sta-password "<PASSWORD>"
   ```

4. Guide them through the next steps:
   - Check the rover's OLED for the "ST" IP address
   - Reconnect their computer to normal Wi-Fi
   - Test with `python wave_rover_controller.py ping --ip <ST_IP>`
