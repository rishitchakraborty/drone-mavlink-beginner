
import time
from pymavlink import mavutil

# Connect to Mission Planner MAVLink Mirror
CONNECTION = "udpin:127.0.0.1:14600"

print("Connecting to Mission Planner SITL...")

vehicle = mavutil.mavlink_connection(CONNECTION)

try:
    vehicle.wait_heartbeat(timeout=10)

    print("\nCONNECTED SUCCESSFULLY!")
    print("System ID:", vehicle.target_system)
    print("Component ID:", vehicle.target_component)

    print("\nReceiving telemetry...\n")

    while True:

        msg = vehicle.recv_match(
            blocking=True,
            timeout=5
        )

        if msg is None:
            print("No MAVLink messages received.")
            continue

        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            continue

        if msg_type == "GLOBAL_POSITION_INT":

            print("\n--- GPS POSITION ---")
            print("Latitude:", msg.lat / 1e7)
            print("Longitude:", msg.lon / 1e7)
            print("Altitude:", msg.relative_alt / 1000, "m")

        elif msg_type == "ATTITUDE":

            print("\n--- ATTITUDE ---")
            print("Roll:", round(msg.roll, 3))
            print("Pitch:", round(msg.pitch, 3))
            print("Yaw:", round(msg.yaw, 3))

        elif msg_type == "SYS_STATUS":

            print("\n--- BATTERY ---")
            print("Voltage:", msg.voltage_battery / 1000, "V")
            print("Battery:", msg.battery_remaining, "%")

        elif msg_type == "VFR_HUD":

            print("\n--- SPEED ---")
            print("Ground speed:", msg.groundspeed, "m/s")
            print("Air speed:", msg.airspeed, "m/s")
            print("Heading:", msg.heading)

except KeyboardInterrupt:
    print("\nTelemetry monitoring stopped.")

except Exception as error:
    print("Error:", error)

finally:
    vehicle.close()
