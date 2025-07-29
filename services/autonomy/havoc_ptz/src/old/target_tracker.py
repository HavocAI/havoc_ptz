import numpy as np
import matplotlib.pyplot as plt


def plot_camera_pointing_vs_boat_heading(boat_pos, target_pos, boat_heading_deg, camera_pan_deg):
    """
    Visualize the boat, its heading, the camera's pointing direction, and the target direction on a 2D plane.
    - boat_pos: (lat, lon, alt) of the boat
    - target_pos: (lat, lon, alt) of the target
    - boat_heading_deg: heading of the boat in degrees (0 = North, increases clockwise)
    - camera_pan_deg: pan angle of the camera relative to boat heading (0 = forward, positive = right)
    """
    import matplotlib.pyplot as plt
    import numpy as np

    # Compute ENU vector from boat to target
    p_boat = geodetic_to_ecef(*boat_pos)
    p_target = geodetic_to_ecef(*target_pos)
    vec = p_target - p_boat
    lat0, lon0, _ = boat_pos
    lat0 = np.radians(lat0)
    lon0 = np.radians(lon0)
    east = np.array([-np.sin(lon0), np.cos(lon0), 0])
    north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
    e = np.dot(vec, east)
    n = np.dot(vec, north)

    # Boat heading: 0 = North, increases clockwise (East = 90)
    # Camera pan: 0 = forward (along heading), positive = right (starboard)
    # Draw boat at (0,0), heading as an arrow
    boat_length = 20
    camera_length = 25
    target_length = np.linalg.norm([e, n])

    # Heading vector (North = 0 deg)
    heading_rad = np.deg2rad(boat_heading_deg)
    heading_vec = np.array([np.sin(heading_rad), np.cos(heading_rad)]) * boat_length

    # Camera pointing vector (relative to heading)
    cam_global_angle = boat_heading_deg + camera_pan_deg
    cam_rad = np.deg2rad(cam_global_angle)
    cam_vec = np.array([np.sin(cam_rad), np.cos(cam_rad)]) * camera_length

    # Target direction vector (from boat to target)
    target_vec = np.array([e, n])
    if target_length > 0:
        target_vec = target_vec / target_length * camera_length

    plt.figure(figsize=(7,7))
    plt.scatter(0, 0, color='blue', s=120, label='Boat')
    plt.arrow(0, 0, heading_vec[0], heading_vec[1], head_width=2, head_length=3, fc='black', ec='black', linewidth=2, label='Boat Heading', length_includes_head=True)
    plt.arrow(0, 0, cam_vec[0], cam_vec[1], head_width=2, head_length=3, fc='red', ec='red', linewidth=2, label='Camera Pointing', length_includes_head=True)
    plt.arrow(0, 0, target_vec[0], target_vec[1], head_width=2, head_length=3, fc='green', ec='green', linestyle=':',linewidth=2, label='Target Direction', length_includes_head=True)
    plt.legend(['Boat','Boat Heading','Camera Pointing','Target Direction'])
    plt.title('Camera Pointing vs Boat Heading and Target')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    margin = camera_length + 10
    plt.xlim(-margin, margin)
    plt.ylim(-margin, margin)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True, linestyle=':')
    plt.show(block=True)


def geodetic_to_ecef(lat, lon, alt):
    # WGS84 ellipsoid constants:
    a = 6378137.0          # semi-major axis
    e2 = 6.69437999014e-3  # first eccentricity squared

    lat = np.radians(lat)
    lon = np.radians(lon)

    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e2) + alt) * np.sin(lat)

    return np.array([x, y, z])


def enu_vector_to_relative_bearing(boat_pos, target_pos, boat_heading_deg):
    """
    Returns the angle (degrees) from the boat heading to the target direction in the ENU plane.
    Positive = target is to the right (starboard), negative = left (port).
    0 = target is straight ahead of the boat heading.
    """
    # Compute ENU vector from boat to target
    p_boat = geodetic_to_ecef(*boat_pos)
    p_target = geodetic_to_ecef(*target_pos)
    vec = p_target - p_boat
    lat0, lon0, _ = boat_pos
    lat0 = np.radians(lat0)
    lon0 = np.radians(lon0)
    east = np.array([-np.sin(lon0), np.cos(lon0), 0])
    north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
    e = np.dot(vec, east)
    n = np.dot(vec, north)
    # Angle of target vector in ENU (0 = North, increases clockwise)
    target_angle = np.degrees(np.arctan2(e, n)) % 360
    # Relative angle to boat heading
    rel_angle = (target_angle - boat_heading_deg + 180) % 360 - 180
    return rel_angle


def compute_tilt_angle(boat_pos, target_pos, boat_pitch_deg):
    """
    Computes the tilt angle required to point at the target from the boat, compensating for the boat's pitch.
    - boat_pos: (lat, lon, alt) of the boat
    - target_pos: (lat, lon, alt) of the target
    - boat_pitch_deg: pitch of the boat in degrees (positive = bow up, negative = bow down)

    Returns:
    - tilt angle in degrees (positive = up, negative = down)
    """
    # Extract altitude differences
    alt_diff = target_pos[2] - boat_pos[2]

    # Compute ENU vector from boat to target
    p_boat = geodetic_to_ecef(*boat_pos)
    p_target = geodetic_to_ecef(*target_pos)
    vec = p_target - p_boat

    # Horizontal distance in EN plane
    lat0, lon0, _ = boat_pos
    lat0 = np.radians(lat0)
    lon0 = np.radians(lon0)
    east = np.array([-np.sin(lon0), np.cos(lon0), 0])
    north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
    e = np.dot(vec, east)
    n = np.dot(vec, north)
    horiz_dist = np.linalg.norm([e, n])

    
    # Elevation angle (from local horizontal up to target)
    elevation = np.degrees(np.arctan2(alt_diff, horiz_dist))

    # Compensate for boat pitch
    tilt_angle = elevation - boat_pitch_deg
    print(horiz_dist, alt_diff,elevation, boat_pitch_deg, tilt_angle)
    return tilt_angle

# Example usage:
target_pos = (45.00776, -83.371729, 100)  
boat_pos = (45.01000, -83.380000, 0)   
boat_heading = -30 # degrees (0 = North, increases clockwise)

pan_angle = enu_vector_to_relative_bearing(boat_pos, target_pos, boat_heading)
print(f"Relative bearing to target: {pan_angle:.2f} degrees")

plot_camera_pointing_vs_boat_heading(boat_pos, target_pos, boat_heading, pan_angle)

boat_pitch = 5  # Boat pitch in degrees (positive = bow up)

# Compute tilt angle
tilt_angle = compute_tilt_angle(boat_pos, target_pos, boat_pitch)
print(f"Tilt angle to target: {tilt_angle:.2f} degrees")