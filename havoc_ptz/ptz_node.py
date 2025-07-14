import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3,PoseStamped
from sensor_msgs.msg import NavSatFix
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from havoc_ptz.msg import CameraState
import tf2_ros
import numpy as np
from builtin_interfaces.msg import Time
from havoc_ptz.viz_vectors import compute_vector_spherical
from havoc_ptz.move import ptzControl

class PTZNode(Node):
    def __init__(self):
        super().__init__('ptz_node')
        self.ptz = ptzControl('192.168.1.42', 80, 'havoc-ptz1', '1nspectR')
        self.origin = (45.03078, -83.40361, 0.0)
        self.rampage_orientation_vector = (0.0,0.0,0.0)    #theta, phi, r
        self.target = None
        self.target_velocity = None
        self.ptz_state = Vector3()
        self.ptz_state.x = 0.0
        self.ptz_state.y = 0.0
        self.ptz_state.z = 1.0
        self.direct_commands_active = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Vector3, '/havoc/abs_ptz_cmd', self.cmd_callback, 10)
        # /havoc/ptz_cmd Vector3 here is desired pan, tilt, zoom values
        self.create_subscription(Vector3, '/havoc/ptz_target_position', self.target_callback, 10)
        self.create_subscription(Vector3, '/havoc/ptz_target_velocity', self.velocity_callback, 10)

        #subscribe to own ship data to correct for own ship position and motion
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.local_position_callback, 10)
        self.create_subscription(Pose, '/mavros/local_position/pose', self.local_orientation_callback, 10)

        self.state_pub = self.create_publisher(Vector3, '/havoc/ptz_state', 10)
        #Vector here is: pan, tilt, zoom
        self.timer = self.create_timer(0.1, self.update_loop)  # 10 Hz
    
    def cmd_callback(self, msg):
        self.direct_commands_active = True
        self.ptz.move_to_absolute(msg.x, msg.y, msg.z)

    def target_callback(self, msg):
        self.direct_commands_active = False
        self.target = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def velocity_callback(self, msg):
        self.target_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def local_position_callback(self, msg):
        # Convert GPS position to ECEF coordinates
        self.origin = (msg.latitude, msg.longitude, 0.0)

    def local_orientation_callback(self, msg):
        # Convert quaternion to spherical coordinates
        theta, phi, r = self.quaternion_to_spherical(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.rampage_orientation_vector = (theta, phi, r)

    def update_state(self):
        # Update the PTZ state based on current pan, tilt, zoom
        state = self.ptz.get_state
        self.ptz_state.x = state['pan']
        self.ptz_state.y = state['tilt']
        self.ptz_state.z = state['zoom']
        return self.ptz_state

    def quaternion_to_spherical(x, y, z, w):
        # Convert quaternion to rotation matrix
        r = R.from_quat([x, y, z, w])
        
        # Rotate the forward vector (0, 0, 1)
        vec = r.apply([0, 0, 1])
        vx, vy, vz = vec

        # Convert to spherical coordinates
        r_mag = np.linalg.norm(vec)
        theta = np.arccos(vz / r_mag)         # inclination from z-axis
        phi = np.arctan2(vy, vx)              # azimuth from x-axis in x-y plane

        return theta, phi, r_mag

    def track_level_to_vector(self, vector):
        #pan should match theta angle 
        #tilt needs to be adjusted so that the camera is level with the horizon
        #Zoom doesnt matter yet
        return 


    def angle_to_ptz_frame(self, theta, phi):
        #Pan: 0 = 0 degrees, 1 = 180 degrees, -1 = -180 degrees 
        #Tilt: 1 = -30, 0 = 30, -1 = 90 degrees
        #Zoom: 0-1

        tilt_factor = 0.016666666 #(2/120 deg)

        ptz_pan = 0.0
        if theta < -180.0:
            pan = -1.0
        elif theta > 180.0:
            pan = 1.0

        ptz_tilt = 0.0
        if phi < -30.0:
            tilt = 1.0
        elif phi > 90.0:
            tilt = -1.0
        else:
            if phi <= 30.0:
                angle = phi - 30.00
                tilt = -1.0 * (angle / 60.0) 
            elif phi > 30.0:
                angle = phi - 30.0
                tilt = -1.0 * (angle / 60.0)

        return pan, tilt, zoom

        return None


    def update_loop(self):
        if self.target is None:
            #update to keep the camera pointed at the rampage orientation vector and publish state
            self.track_level_to_vector(self.rampage_orientation_vector)
            self.update_state()
            self.state_pub.publish(self.ptz_state)
            return
        else:
            target = self.target.copy()
            if self.target_velocity is not None:
                target_vel = self.target_velocity.copy()  # Predict 0.5s ahead
                target += 0.5 * target_vel

            theta, phi, r = compute_vector_spherical(*self.origin, *target)
            pan = theta / 180.0
            tilt = 1.0 - phi / 180.0
            self.ptz.move_to_absolute(pan, tilt, 1.0)

        self.update_state()

        self.state_pub.publish(self.ptz_state)

def main(args=None):
    rclpy.init(args=args)
    node = PTZNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
