
import time
import math
from pymavlink import mavutil

# ==========================================
# CONFIGURATION
# ==========================================

CONNECTION = "udpin:0.0.0.0:14600"

TAKEOFF_ALTITUDE = 5
MAX_ALTITUDE = 30
MAX_ROUTE_RADIUS = 500  # metres from starting position

# Example coordinates.
# Replace these with coordinates near your SITL home location.
WAYPOINTS = [
    (22.572700, 88.363900, 5),
    (22.572800, 88.364000, 7),
    (22.572900, 88.363900, 7),
]

# ==========================================
# CONNECT
# ==========================================

vehicle = mavutil.mavlink_connection(CONNECTION)

print("Waiting for heartbeat...")
vehicle.wait_heartbeat(timeout=30)

print("Connected!")
print("System:", vehicle.target_system)

# ==========================================
# READ GPS
# ==========================================

def get_position():

    msg = vehicle.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=10
    )

    if msg is None:
        raise TimeoutError("GPS position unavailable")

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000

    return lat, lon, alt


# ==========================================
# DISTANCE CALCULATION
# ==========================================

def distance_m(lat1, lon1, lat2, lon2):

    R = 6371000

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)

    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = (
        math.sin(dp / 2) ** 2
        + math.cos(p1) * math.cos(p2)
        * math.sin(dl / 2) ** 2
    )

    return 2 * R * math.atan2(
        math.sqrt(a),
        math.sqrt(1 - a)
    )


# ==========================================
# VALIDATE ROUTE
# ==========================================

def validate_route(home_lat, home_lon):

    for index, (lat, lon, alt) in enumerate(WAYPOINTS):

        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("Invalid GPS coordinates")

        if not (0 < alt <= MAX_ALTITUDE):
            raise ValueError("Invalid waypoint altitude")

        distance = distance_m(
            home_lat, home_lon, lat, lon
        )

        if distance > MAX_ROUTE_RADIUS:
            raise ValueError(
                f"Waypoint {index + 1} exceeds route radius"
            )

        print(
            f"Waypoint {index + 1}: "
            f"{lat}, {lon}, {alt}m "
            f"({distance:.1f}m from home)"
        )


# ==========================================
# BUILD MAVLINK MISSION
# ==========================================

def create_item(seq, command, frame, x, y, z,
                p1=0, p2=0, p3=0, p4=0):

    return mavutil.mavlink.MAVLink_mission_item_int_message(
        vehicle.target_system,
        vehicle.target_component,
        seq,
        frame,
        command,
        0,
        1,
        p1, p2, p3, p4,
        x, y, z,
        0
    )


def build_mission():

    frame = (
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    )

    mission = []

    # Takeoff item
    mission.append(
        create_item(
            0,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            frame,
            0, 0,
            TAKEOFF_ALTITUDE
        )
    )

    # Waypoint items
    for lat, lon, alt in WAYPOINTS:

        mission.append(
            create_item(
                len(mission),
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                frame,
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                p1=2
            )
        )

    # Return to launch
    mission.append(
        create_item(
            len(mission),
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            frame,
            0, 0, 0
        )
    )

    return mission


# ==========================================
# UPLOAD MISSION
# ==========================================

def upload_mission(mission):

    print("Clearing previous mission...")

    vehicle.mav.mission_clear_all_send(
        vehicle.target_system,
        vehicle.target_component
    )

    response = vehicle.recv_match(
        type="MISSION_ACK",
        blocking=True,
        timeout=10
    )

    if response is None or response.type != 0:
        raise RuntimeError("Mission clear was not confirmed")

    print("Sending mission count:", len(mission))

    vehicle.mav.mission_count_send(
        vehicle.target_system,
        vehicle.target_component,
        len(mission)
    )

    sent = set()

    while len(sent) < len(mission):

        msg = vehicle.recv_match(
            type=[
                "MISSION_REQUEST_INT",
                "MISSION_REQUEST",
                "MISSION_ACK"
            ],
            blocking=True,
            timeout=15
        )

        if msg is None:
            raise TimeoutError("Mission upload timed out")

        if msg.get_type() == "MISSION_ACK":

            if msg.type != 0:
                raise RuntimeError(
                    f"Mission rejected: {msg.type}"
                )

            if len(sent) == len(mission):
                print("Mission upload confirmed!")
                return

            raise RuntimeError("Unexpected mission ACK")

        seq = msg.seq

        if not 0 <= seq < len(mission):
            raise RuntimeError("Invalid mission sequence")

        item = mission[seq]

        if msg.get_type() == "MISSION_REQUEST_INT":

            vehicle.mav.send(item)

        else:
            # Older MAVLink mission request
            vehicle.mav.mission_item_send(
                vehicle.target_system,
                vehicle.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x / 1e7,
                item.y / 1e7,
                item.z
            )

        sent.add(seq)
        print(f"Uploaded waypoint item {seq}")


# ==========================================
# MAIN
# ==========================================

try:

    print("\nReading current GPS...")

    home_lat, home_lon, home_alt = get_position()

    print("Starting position:")
    print(home_lat, home_lon, home_alt)

    print("\nValidating route...")

    validate_route(home_lat, home_lon)

    mission = build_mission()

    print("\nMission preview:")

    for item in mission:
        print(
            "Sequence:", item.seq,
            "Command:", item.command,
            "Coordinates:", item.x, item.y,
            "Altitude:", item.z
        )

    confirmation = input(
        "\nUpload this mission to SITL? Type YES: "
    )

    if confirmation == "YES":

        upload_mission(mission)

        print(
            "\nMission uploaded. "
            "Review it in Mission Planner before execution."
        )

    else:
        print("Mission upload cancelled.")

finally:
    vehicle.close()
    print("Connection closed.")
