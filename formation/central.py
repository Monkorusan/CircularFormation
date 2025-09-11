import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class CentralNode(Node):
    def __init__(self):
        super().__init__(
            'central',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.N = 6
        self.RR = 2.0
        self.rc = [6.0] * 6
        self.w = 2.0
        self.q = 6.0



        self.m = 3




        self.dt = 0.1
        #self.k = 5.76 #11pi/6
        self.k = np.pi / 6
        self.l = 1

        self.e = 0.5
        self.kv = 0.25
        self.kw = 0.1

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
        
        self.h = (2 * np.pi * self.l + (self.m - self.N) * self.k) / self.m
        
        if self.m == 1:
            self.B = [self.k, self.k, self.k, self.k, self.k, self.h]

        elif self.m == 2:
            self.B = [self.k, self.k, self.h, self.k, self.k, self.h]

        elif self.m == 3:
            self.B = [self.k, self.h, self.k, self.h, self.k, self.h]

        else:
            self.B = [self.k] * self.N

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

    def timer_callback(self): #this function is called every dt seconds

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
