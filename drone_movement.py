
import time
from pymavlink import mavutil

# ==========================================
# SITL CONNECTION
# ==========================================

# Direct bidirectional SITL connection.
# This is separate from the UDP telemetry mirror.
CONNECTION = "udpin:0.0.0.0:14600"

TAKEOFF_ALTITUDE = 20.0
FORWARD_SPEED = 10.0
MOVEMENT_DURATION = 10.0

vehicle = None


# ==========================================
# CONNECT
# ==========================================

def connect_drone():

    global vehicle

    print("Connecting to SITL...")

    vehicle = mavutil.mavlink_connection(CONNECTION)

    vehicle.wait_heartbeat(timeout=30)

    print("Connected!")
    print("System ID:", vehicle.target_system)
    print("Component ID:", vehicle.target_component)

    if vehicle.target_system == 0:
        raise RuntimeError(
            "Invalid autopilot system ID. "
            "Check the direct SITL connection."
        )


# ==========================================
# CHANGE MODE
# ==========================================

def set_guided_mode():

    print("\nChanging mode to GUIDED...")

    mode_id = vehicle.mode_mapping()["GUIDED"]

    vehicle.set_mode(mode_id)

    time.sleep(2)

    print("GUIDED mode requested.")


# ==========================================
# ARM
# ==========================================

def arm_drone():

    print("\nArming motors...")

    vehicle.arducopter_arm()

    vehicle.motors_armed_wait()

    print("Motors armed!")


# ==========================================
# TAKEOFF
# ==========================================

def takeoff():

    print("\nTaking off...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0,
        0, 0, 0,
        TAKEOFF_ALTITUDE
    )

    # Wait until target altitude is reached
    start = time.time()

    while time.time() - start < 30:

        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=2
        )

        if msg is None:
            continue

        altitude = msg.relative_alt / 1000.0

        print(f"Current altitude: {altitude:.2f} m")

        if altitude >= TAKEOFF_ALTITUDE * 0.90:
            print("Takeoff altitude reached!")
            return

    raise TimeoutError("Takeoff altitude not reached.")


# ==========================================
# MOVE FORWARD / BACKWARD
# ==========================================

def move_velocity(speed, duration):

    # Velocity-only control in body coordinates.
    # Positive X = forward
    # Negative X = backward
    # Zero velocity = hover

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    print(f"\nMoving at {speed} m/s for {duration} seconds")

    start = time.time()

    while time.time() - start < duration:

        vehicle.mav.set_position_target_local_ned_send(
            0,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            speed, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(0.1)

    stop_drone()


# ==========================================
# STOP / HOVER
# ==========================================

def stop_drone():

    print("Stopping movement...")

    move_velocity_zero = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    for _ in range(10):

        vehicle.mav.set_position_target_local_ned_send(
            0,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            move_velocity_zero,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(0.1)

    print("Zero velocity command sent.")


# ==========================================
# LAND
# ==========================================

def land_drone():

    print("\nLanding drone...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0,
        0, 0, 0,
        0
    )

    print("Landing command sent.")


# ==========================================
# MAIN FLIGHT SEQUENCE
# ==========================================

try:

    connect_drone()

    set_guided_mode()

    arm_drone()

    takeoff()

    # Move forward
    move_velocity(
        FORWARD_SPEED,
        MOVEMENT_DURATION
    )

    print("\nHovering...")
    time.sleep(2)

    # Move backward
    move_velocity(
        -FORWARD_SPEED,
        MOVEMENT_DURATION
    )

    print("\nHovering...")
    time.sleep(2)

    # Land
    land_drone()

    print("\nFlight sequence completed.")

except KeyboardInterrupt:

    print("\nInterrupted! Requesting landing...")

    if vehicle is not None:
        try:
            land_drone()
        except Exception:
            print("Could not send landing command.")

except Exception as error:

    print("\nERROR:", error)

    print("Do not assume the drone has landed.")
    print("Check SITL status and land using Mission Planner.")

finally:

    if vehicle is not None:
        vehicle.close()

    print("\nConnection closed.")
