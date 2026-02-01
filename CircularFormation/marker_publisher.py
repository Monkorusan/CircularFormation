import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import numpy as np

# ---------------- Quaternion utilities ---------------- #
def quaternion_from_axis_angle(axis, angle_rad):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    s = np.sin(angle_rad / 2.0)
    x, y, z = axis * s
    w = np.cos(angle_rad / 2.0)
    return x, y, z, w

def quaternion_multiply(q1, q2):
    """Hamilton product q = q1 * q2 (x, y, z, w order)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def get_yaw(q_msg):
    """Return yaw (heading) from geometry_msgs/Quaternion."""
    siny = 2.0 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y)
    cosy = 1.0 - 2.0 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z)
    return np.arctan2(siny, cosy)

def wheel_quaternion_global(pose):
    """Cylinder axis along robot‑X → 90 deg about local +Y then apply robot pose."""
    robot_q = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    wheel_local_q = quaternion_from_axis_angle((0, 1, 0), np.pi / 2)  # lay cylinder on side
    return quaternion_multiply(robot_q, wheel_local_q)

def rotated_offset(pose, dx, dy):
    """(dx, dy) in robot frame → (x, y) in world frame."""
    yaw = get_yaw(pose.orientation)
    x_off = dx * np.cos(yaw) - dy * np.sin(yaw)
    y_off = dx * np.sin(yaw) + dy * np.cos(yaw)
    return pose.position.x + x_off, pose.position.y + y_off

# ---------------- Marker publisher node ---------------- #
class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        self.pub = self.create_publisher(MarkerArray, "robot_markers", 10)
        self.robot_poses = {}

        # Subscribe to up to six robot pose topics
        for i in range(6):
            self.create_subscription(
                PoseStamped, f"/robot_{i}/pose", lambda msg, idx=i: self.pose_cb(msg, idx), 10
            )

        # One‑shot delete‑all to clear any old markers when RViz restarts
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_arr = MarkerArray()
        delete_arr.markers.append(delete_all)
        self.pub.publish(delete_arr)

        self.timer = self.create_timer(0.1, self.publish_markers)

    def pose_cb(self, msg, idx): #this function is called when a new pose is published
        self.robot_poses[idx] = msg.pose

    def publish_markers(self): #this function is called every 0.1 seconds
        if not self.robot_poses:
            return

        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, pose in self.robot_poses.items():
            base_id = i * 2  # body + arrow

            # ---------- Body ----------
            body = Marker()
            body.header.frame_id = "map"
            body.header.stamp = stamp
            body.ns = f"robot_{i}"
            body.id = base_id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose = pose
            body.pose.position.z = 0.25
            body.scale.x = body.scale.y = 0.5
            body.scale.z = 0.5
            body.color.a = 1.0
            body.color.r, body.color.g, body.color.b = 0.0, 1.0, 0.0

            # ---------- Facing Arrow(to show theta) ----------
            arrow = Marker()
            arrow.header = body.header
            arrow.ns = body.ns
            arrow.id = base_id + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose  # Same orientation as robot body
            arrow.scale.x = 0.6  # Arrow length
            arrow.scale.y = 0.08  # Arrow shaft diameter
            arrow.scale.z = 0.12  # Arrow head diameter
            arrow.color.a = 1.0
            arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.0, 0.0  # Red arrow

            arr.markers.append(body)
            arr.markers.append(arrow)

        self.pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": 
    main() 