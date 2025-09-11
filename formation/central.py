import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class CentralNode(Node):
    def __init__(self):
        super().__init__('central',)

        # reminder: only N,m and l are type-int, the rest are type-float.
        self.declare_parameter('N', 6) #robot num
        self.declare_parameter('RR', 2.0) #init radius
        self.declare_parameter('rc', [6.0] * 6) #convergence radius
        self.declare_parameter('w', 2.0) # init phase numerator
        self.declare_parameter('q', 6.0) # init phase denominator
        self.declare_parameter('m', 3) # cluster num
        self.declare_parameter('dt', 0.1) # derivative time gap
        self.declare_parameter('k', float(np.pi / 6)) #bias coeff for in-cluster phase diff
        self.declare_parameter('l', 1) # pattern l
        self.declare_parameter('e', 0.5) # coupling strength
        self.declare_parameter('kv', 0.25) # velocity gain
        self.declare_parameter('kw', 0.1) # angular velocity gain

        #access yaml file's falues. 
        self.N = self.get_parameter('N').value
        self.RR = self.get_parameter('RR').value
        self.rc = self.get_parameter('rc').value
        self.w = self.get_parameter('w').value
        self.q = self.get_parameter('q').value
        self.m = self.get_parameter('m').value
        self.dt = self.get_parameter('dt').value
        self.k = self.get_parameter('k').value
        self.l = self.get_parameter('l').value
        self.e = self.get_parameter('e').value
        self.kv = self.get_parameter('kv').value
        self.kw = self.get_parameter('kw').value

        # -------------------------
        # Initialize states
        # -------------------------
        self.t = 0.0
        self.y = []
        for i in range(self.N): #assume uniform dist for init phi
            phi_i = (self.N - 1 - i) * self.w * np.pi / self.q
            self.y.extend([self.RR, phi_i, np.pi / 2])
        self.y = np.array(self.y)

        # -------------------------
        # Coupling phase shift B
        # -------------------------

        N = self.N
        m = self.m
        k = self.k
        l = self.l
        h = (2 * np.pi * l + (m - N) * k) / m
        B = None  # Temporary local variable
        
        #as of now, only case N==6 is considered
        if m == 1:
            B = [k, k, k, k, k, h]
        elif m == 2:
            B = [k, k, h, k, k, h]
        elif m == 3:
            B = [k, h, k, h, k, h]
        else:
            B = [k] * N

        self.B = B  # Assign once at the end

        # -------------------------
        # Create publishers for robot poses
        # -------------------------
        self.publishers_list = [
            self.create_publisher(PoseStamped, f'/robot_{i}/pose', 10)
            for i in range(self.N)
        ]

        # -------------------------
        # Start simulation timer
        # -------------------------
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self): 

        """
        this function is called every dt seconds passes to indicate that a time step has passed.
        a common practice in ROS2 coding.
        """

        dy = self.robot_dynamics(self.y, self.rc, self.B, self.N)
        self.y += self.dt * dy
        self.t += self.dt

        r = self.y[::3]
        phi = self.y[1::3]
        theta = self.y[2::3]

        x = r * np.cos(phi)
        y = r * np.sin(phi)

        for i in range(self.N):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(x[i])
            msg.pose.position.y = float(y[i])
            msg.pose.position.z = 0.0

            # Set orientation based on phi and theta
            yaw = phi[i] + theta[i]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = np.sin(yaw / 2.0)
            msg.pose.orientation.w = np.cos(yaw / 2.0)

            self.publishers_list[i].publish(msg)

    def robot_dynamics(self, y, rc, B, N):
        """
        THe core logic for our project stays here.
        """
        r = y[::3]
        phi = y[1::3]
        theta = y[2::3]

        rdot = np.zeros(N)
        phidot = np.zeros(N)
        thetadot = np.zeros(N)

        e = self.e
        kv = self.kv
        kw = self.kw

        for a in range(N):
            f = r[a] * (1 - (r[a] / rc[a]) ** 2)
            if a == N - 1:
                g = 1 + e * np.sin(phi[0] - phi[a] + B[a])
            else:
                g = 1 + e * np.sin(phi[a + 1] - phi[a] + B[a])

            v = kv * (f * np.cos(theta[a]) + r[a] * g * np.sin(theta[a]))
            omega = kw * (r[a] * g * np.cos(theta[a]) - f * np.sin(theta[a]))

            rdot[a] = v * np.cos(theta[a])
            phidot[a] = (1 / r[a]) * v * np.sin(theta[a])
            thetadot[a] = omega

        dy = np.zeros(3 * N)
        dy[::3] = rdot
        dy[1::3] = phidot
        dy[2::3] = thetadot
        return dy


def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
