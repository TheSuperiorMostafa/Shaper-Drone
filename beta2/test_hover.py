# test_hover.py
"""
Simple test flight script for Flywoo GOKU GN745 (ArduPilot).

Behavior:
- Connect to the flight controller over serial
- Manual command loop for arm / mode / takeoff / land / status
- Enter triggers emergency disarm
- Ctrl+C triggers emergency disarm immediately
- R uses rangefinder-based takeoff

Run on the Raspberry Pi that is wired to the GN745:
    python3 test_hover_gn745.py

Make sure safety procedures are in place (RC transmitter, kill switch, safety net).
"""

import time

import navigation
from pymavlink import mavutil


TARGET_ALTITUDE_M = 0.15
TARGET_ALTITUDE_M_RANGEFINDER = 0.3
FC_PORT = '/dev/serial0'
FC_BAUD = 115200


def main():
    print("[TEST] === GN745 hover test starting ===")
    fc = navigation.FlightController(FC_PORT, FC_BAUD)
    if fc is None:
        print("[TEST] Aborting: no flight controller connection.")
        return

    try:
        # Connect to FC
        fc.connect_to_fc()

        # Ask user if we should set EKF origin
        setekf = input(
            "[TEST] Set EKF origin? This should be done only ONCE, when the drone first turns on. "
            "Type y or Y and press enter.\n"
        ).strip().lower()

        if setekf == "y":
            print("[TEST] Setting EKF origin")
            fc.set_EKF_origin()
        else:
            print("[TEST] EKF origin setting skipped!")

        print("[TEST] Entering main control loop")

        exit_flight = False
        while not exit_flight:
            drone_command = input(
                "Enter control input (either uppercase/lowercase): "
                "A to arm | D to disarm | G to enter guided mode | "
                "T to takeoff | R to takeoff_with_rangefinder | "
                "L to land | S for status | E to exit | "
                "Enter or anything else for EMERGENCY CUTOFF!\n"
            ).strip().lower()

            match drone_command:
                case 'a':  # arm
                    fc.arm()

                case 'd':  # disarm
                    fc.disarm()

                case 'g':  # guided mode
                    fc.set_guided_mode()

                case 't':  # normal takeoff
                    fc.takeoff(altitude=TARGET_ALTITUDE_M)

                case 'r':  # rangefinder-based takeoff
                    fc.takeoff_with_rangefinder(TARGET_ALTITUDE_M_RANGEFINDER)

                case 's':  # status
                    fc.update_vehicle_state()
                    z_ned = fc.vehicle_state['position']['z']
                    altitude_m = -z_ned
                    armed = fc.vehicle_state['armed']
                    mode = fc.vehicle_state['mode']

                    print(f"[TEST] Armed: {armed}")
                    print(f"[TEST] Mode: {mode}")
                    print(f"[TEST] Estimated altitude: {altitude_m:.2f} m (z_ned = {z_ned:.2f})")

                case 'l':  # land
                    fc.land()
                
                case 'm':
                    fc.debug_request_rangefinder(3.0)
                    fc.debug_dump_messages(5.0)

                case 'e':  # exit
                    confirm = input(
                        "Are you sure you want to exit? Enter Y\n"
                        "Only do this on the ground, it will terminate your control of the drone!\n"
                    ).strip().lower()

                    if confirm == 'y':
                        print("[TEST] Terminating program")
                        exit_flight = True

                case _:  # emergency cutoff, including blank Enter
                    print("[TEST] EMERGENCY CUTOFF ACTIVATED!")
                    fc.emergency_disarm()

    except KeyboardInterrupt:
        print("\n[TEST] Ctrl+C detected - EMERGENCY CUTOFF ACTIVATED!")
        try:
            fc.emergency_disarm()
            time.sleep(0.5)
        except Exception as e:
            print(f"[TEST] Failed to send emergency disarm: {e}")

    finally:
        print("[TEST] Closing connection to flight controller")
        fc.close()


if __name__ == "__main__":
    main()