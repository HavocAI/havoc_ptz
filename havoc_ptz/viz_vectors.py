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
    vec = p2 - p1
    theta, phi, r = cartesian_to_spherical(vec)
    return theta, phi, r



# Example usage:
point2 = (45.03372, -83.37479, 0)  # San Francisco, sea level
point1 = (45.03078, -83.40361, 0)   # New York, sea level

theta, phi, r = compute_vector_spherical(point1[0], point1[1], point1[2], point2[0], point2[1], point2[2])
print(f"Theta (azimuth): {theta:.2f}°")
print(f"Phi (elevation): {phi:.2f}°")
print(f"Distance r: {r:.2f} m")

vector_spherical = (theta, phi, r)  # theta=60°, phi=45°, r=1000km
plot_points_and_vector_on_globe(point1, point2, vector_spherical)