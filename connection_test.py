import time
from pymavlink import mavutil


# Connect to Mission Planner MAVLink Mirror
CONNECTION = "udpin:127.0.0.1:14600"

# Connect to Mission Planner's MAVLink TCP host
vehicle = mavutil.mavlink_connection(
    CONNECTION
)

# # Connect to Mission Planner's MAVLink TCP host
# vehicle = mavutil.mavlink_connection(
#     "tcp:127.0.0.1:5760"
# )

print("Waiting for the simulated drone...")
vehicle.wait_heartbeat(timeout=10)

print(
    f"Connected! System ID: {vehicle.target_system}, "
    f"Component ID: {vehicle.target_component}"
)

# Display live position and altitude
try:
    while True:
        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=5
        )

        if msg is not None:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.relative_alt / 1000.0

            print(
                f"Lat: {latitude:.6f}, "
                f"Lon: {longitude:.6f}, "
                f"Altitude: {altitude:.1f} m"
            )

except KeyboardInterrupt:
    print("Telemetry monitoring stopped.")

finally:
    vehicle.close()