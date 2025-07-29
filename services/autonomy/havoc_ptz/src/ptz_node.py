import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, Pose
import numpy as np
from havoc_ptz.ptz_controller import ptzControl

class PTZNode(Node):
    def __init__(self):
        super().__init__('ptz_node')
        self.ptz = ptzControl('192.168.1.42', 80, 'havoc-ptz1', '1nspectR')
        self.boat_position = (0.0, 0.0, 0.0)  # (lat, lon, alt)
        self.boat_heading = 0.0  # degrees
        self.boat_pitch = 0.0  # degrees
        self.boat_roll = 0.0 # degrees
        self.target_position = None  # (lat, lon, alt)

        # Subscriptions
        self.create_subscription(Vector3, '/havoc/ptz_target_position', self.target_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.boat_position_callback, 10)
        self.create_subscription(Pose, '/mavros/local_position/pose', self.boat_orientation_callback, 10)

        # Timer for periodic updates
        self.timer = self.create_timer(0.5, self.update_loop)  # 2 Hz

    def target_callback(self, msg):
        self.target_position = (msg.x, msg.y, msg.z)

    def boat_position_callback(self, msg):
        self.boat_position = (msg.latitude, msg.longitude, msg.altitude)

    def boat_orientation_callback(self, msg):
        # Convert quaternion to heading and pitch
        q = msg.orientation
        heading, pitch, roll = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.boat_heading = np.degrees(heading)
        self.boat_pitch = np.degrees(pitch)
        self.boat_roll = np.degrees(roll)

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to NED Euler angles (heading, pitch, roll)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return yaw, pitch, roll

    def update_loop(self):
        if self.target_position is None:
            return

        # Track the target using ptzControl
        self.ptz.track_target(
            target_position=self.target_position,
            boat_position=self.boat_position,
            boat_heading_deg=self.boat_heading,
            boat_pitch_deg=self.boat_pitch,
            boat_roll_deg=self.boat_roll,
            zoom_percent=10.0  # Example zoom level
        )

def main(args=None):
    rclpy.init(args=args)
    node = PTZNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
