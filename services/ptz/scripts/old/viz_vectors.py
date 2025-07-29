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
    plt.arrow(0, 0, target_vec[0], target_vec[1], head_width=2, head_length=3, fc='green', ec='green', linewidth=2, label='Target Direction', length_includes_head=True)
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

def plot_points_and_vector_2d(point1, point2):
    """
    Plots two geodetic points and the vector between them on a flat 2D plane (local tangent plane).
    The plot is zoomed in to show the points and vector clearly.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    # Convert geodetic to ECEF
    p1_ecef = geodetic_to_ecef(*point1)
    p2_ecef = geodetic_to_ecef(*point2)
    # Use p1 as the origin, plot the vector from p1 to p2 in the local tangent plane
    vec = p2_ecef - p1_ecef

    # Define local ENU (East-North-Up) axes at p1
    lat0, lon0, _ = point1
    lat0 = np.radians(lat0)
    lon0 = np.radians(lon0)
    # ENU basis vectors
    east = np.array([-np.sin(lon0), np.cos(lon0), 0])
    north = np.array([-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)])
    # Project vector onto ENU
    e = np.dot(vec, east)
    n = np.dot(vec, north)

    # Plot
    plt.figure(figsize=(6,6))
    plt.scatter(0, 0, color='red', s=100, label='Point 1 (origin)')
    plt.scatter(e, n, color='green', s=100, label='Point 2')
    plt.arrow(0, 0, e, n, head_width=5, head_length=5, fc='purple', ec='purple', linewidth=2, length_includes_head=True, label='Vector')
    plt.plot([0, e], [0, n], color='orange', linestyle='--', label='Between Points')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('2D Local Tangent Plane (Zoomed)')
    plt.legend()
    # Set axis limits to zoom in
    margin = 0.2 * np.linalg.norm([e, n]) + 10
    plt.xlim(min(0, e) - margin, max(0, e) + margin)
    plt.ylim(min(0, n) - margin, max(0, n) + margin)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True, linestyle=':')
    plt.show(block=True)
import numpy as np
import matplotlib.pyplot as plt

def latlonalt_to_xyz(lat, lon, alt, R=6371000):
    # Convert degrees to radians
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    r = R + alt
    x = r * np.cos(lat_rad) * np.cos(lon_rad)
    y = r * np.cos(lat_rad) * np.sin(lon_rad)
    z = r * np.sin(lat_rad)
    return np.array([x, y, z])

def spherical_to_xyz(theta, phi, r):
    # theta: azimuthal (deg, from x-axis in x-y plane)
    # phi: polar (deg, from z-axis)
    theta_rad = np.deg2rad(theta)
    phi_rad = np.deg2rad(phi)
    x = r * np.sin(phi_rad) * np.cos(theta_rad)
    y = r * np.sin(phi_rad) * np.sin(theta_rad)
    z = r * np.cos(phi_rad)
    return np.array([x, y, z])

def plot_points_and_vector_on_globe(
    point1, point2, vector_spherical, earth_radius=6371000
):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the sphere
    u, v = np.mgrid[0:2*np.pi:100j, 0:np.pi:100j]
    x = earth_radius * np.cos(u) * np.sin(v)
    y = earth_radius * np.sin(u) * np.sin(v)
    z = earth_radius * np.cos(v)
    ax.plot_surface(x, y, z, color='lightblue', alpha=0.3, linewidth=0)

    # Convert points to xyz
    p1_xyz = latlonalt_to_xyz(*point1, R=earth_radius)
    p2_xyz = latlonalt_to_xyz(*point2, R=earth_radius)

    # Plot the points
    ax.scatter(*p1_xyz, color='red', s=100, label='Point 1')
    ax.scatter(*p2_xyz, color='green', s=100, label='Point 2')

    # Plot the vector from point 1
    vec_xyz = spherical_to_xyz(*vector_spherical)
    ax.quiver(
        p1_xyz[0], p1_xyz[1], p1_xyz[2],
        vec_xyz[0], vec_xyz[1], vec_xyz[2],
        color='purple', length=1, normalize=False, linewidth=2, arrow_length_ratio=0.1, label='Vector'
    )

    # Optionally, plot the line between the two points
    ax.plot([p1_xyz[0], p2_xyz[0]], [p1_xyz[1], p2_xyz[1]], [p1_xyz[2], p2_xyz[2]], color='orange', linestyle='--', label='Between Points')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_box_aspect([1,1,1])
    plt.title('Points and Vector on Globe')
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

def cartesian_to_spherical(vec):
    x, y, z = vec
    r = np.linalg.norm(vec)
    theta = np.arctan2(y, x)          # azimuth
    phi = np.arccos(z / r)            # polar angle from z-axis

    return np.degrees(theta), np.degrees(phi), r

def compute_vector_spherical(lat1, lon1, alt1, lat2, lon2, alt2):
    p1 = geodetic_to_ecef(lat1, lon1, alt1)
    p2 = geodetic_to_ecef(lat2, lon2, alt2)
    print(p1,p2)
    vec = p2 - p1
    print(f"Vector: {vec}")
    theta, phi, r = cartesian_to_spherical(vec)
    return theta, phi, r



# Example usage:
point2 = (45.00776, -83.371729, 0)  # 
point1 = (45.01000, -83.380000, 0)   # Point

theta, phi, r = compute_vector_spherical(point1[0], point1[1], point1[2], point2[0], point2[1], point2[2])
print(f"Theta (azimuth): {theta:.2f}°")
print(f"Phi (elevation): {phi:.2f}°")
print(f"Distance r: {r:.2f} m")

vector_spherical = (theta, phi, r)  # theta=60°, phi=45°, r=1000km
plot_points_and_vector_on_globe(point1, point2, vector_spherical)

# 2D visualization (zoomed in)
plot_points_and_vector_2d(point1, point2)

# Example: visualize camera pointing vs boat heading and target
# Assume boat heading is 30 deg (NE), camera pan is 10 deg right of heading
boat_heading = 30  # degrees (0 = North)
#camera_pan = 10    # degrees (relative to heading)
plot_camera_pointing_vs_boat_heading(point1, point2, boat_heading, camera_pan)