# test_hover.py
"""
Simple test flight script for Flywoo GOKU GN745 (ArduPilot).

Behavior:
- Connect to the flight controller over serial
- Take off to a fixed altitude
- Hover for a few seconds
- Command a land and wait until back on the ground

Run on the Raspberry Pi that is wired to the GN745:
    python3 test_hover_gn745.py

Make sure safety procedures are in place (RC transmitter, kill switch, safety net).
"""

import time

import navigation 
from pymavlink import mavutil


TARGET_ALTITUDE_M = 1.5      # meters above home for the hover
FC_PORT = '/dev/serial0'
FC_BAUD = 115200

def main():
    print("[TEST] === GN745 hover test starting ===")
    fc = navigation.FlightController(FC_PORT, FC_BAUD)
    if fc is None:
        print("[TEST] Aborting: no flight controller connection.")
        return

    try:
        # connect to FC
        fc.connect_to_fc()

        # ask user if we should set EKF origin
        setekf = input("[TEST] Set EKF origin? This should be done only ONCE, when the drone first turns on. Type y or Y and press enter.\n").lower()
        if setekf == "y":
            print("[TEST] Setting EKF origin")
            fc.set_EKF_origin()
        else:
            print("[TEST] EKF origin setting skipped!")
        
        # ask user to arm the drone
        print("[TEST] Entering main control loop")

        exit_flight = False
        while (exit_flight == False):
            drone_command = input("Enter control input (either uppercase/lowercase): "
                      "A to arm | D to disarm | G to enter guided mode | T to takeoff | L to land | E to exit | anything else for EMERGENCY CUTOFF!\n")

            # deciding what to do with the input
            match drone_command.lower():
                case 'a': # arm
                    fc.arm()
                case 'd': # disarm
                    fc.disarm()
                case 'g': # enter guided mode
                    fc.set_guided_mode()
                case 't': # takeoff
                    fc.takeoff(altitude = TARGET_ALTITUDE_M)
                case 'l': # land
                    fc.land()
                case 'e': # exit control loop
                    if input("Are you sure you want to exit? Enter Y\n Only do this on the ground, it will terminate your control of the drone!\n").lower() == 'y':
                        print("Terminating program")
                        fc.close()
                        exit_flight = True
                case _: # emergency disarm
                    print("EMERGENCY CUTOFF ACTIVATED!")
                    fc.emergency_disarm()

    finally:
        fc.close()


if __name__ == "__main__":
    main()