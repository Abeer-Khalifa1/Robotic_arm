import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math
import time
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(-math.pi, math.pi)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits
        self._integral = 0.0
        self._previous_error = 0.0

    def update(self, setpoint, current_value, dt):
        error = setpoint - current_value
        p_term = self.Kp * error
        self._integral += error * dt
        self._integral = max(min(self._integral, self.output_limits[1]), self.output_limits[0])
        i_term = self.Ki * self._integral
        derivative = (error - self._previous_error) / dt if dt > 0 else 0.0
        d_term = self.Kd * derivative
        self._previous_error = error
        output = p_term + i_term + d_term
        return max(min(output, self.output_limits[1]), self.output_limits[0])

class CartesianJacobianPIDController(Node):
    def __init__(self):
        super().__init__('jacobian_pid_controller')

        self.joint_pubs = [self.create_publisher(Float64, f'/joint_{i}/position_cmd', 10) for i in range(5)]
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_joint_angles = [0.0] * 5
        self.joint_names = [f'joint_{i}' for i in range(5)]

        self.pid_controllers = [
            PIDController(1, 0.00001, 0.2),
            PIDController(1, 0.00001, 0.2),
            PIDController(1, 0.00001, 0.2),
            PIDController(1, 0.00001, 0.2),
            PIDController(1, 0.00001, 0.2)
        ]

        self.L1, self.L2, self.L3, self.L4 = 110.0, 170.0, 62.0, 100.0
        self.L34 = self.L3 + self.L4

        self.start_time = time.time()
        self.square_path = self.generate_square_path_xy(160.0, 180.0, 10.0, 100.0)
        self.path_segments = self.generate_cartesian_segments(self.square_path, steps_per_edge=40)
        self.segment_index = 0

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.update)

    def joint_state_callback(self, msg):
        temp = {name: pos for name, pos in zip(msg.name, msg.position)}
        for i, name in enumerate(self.joint_names):
            self.current_joint_angles[i] = temp.get(name, 0.0)

    def generate_square_path_xy(self, fixed_z, center_x, center_y, side_length):
        half = side_length / 2.0
        return [
            [center_x - half, center_y - half, fixed_z],
            [center_x + half, center_y - half, fixed_z],
            [center_x + half, center_y + half, fixed_z],
            [center_x - half, center_y + half, fixed_z],
            [center_x - half, center_y - half, fixed_z],
        ]

    def generate_cartesian_segments(self, path, steps_per_edge):
        segments = []
        for i in range(len(path) - 1):
            p0, p1 = path[i], path[i + 1]
            for step in range(steps_per_edge):
                alpha = step / steps_per_edge
                point = [(1 - alpha) * p0[j] + alpha * p1[j] for j in range(3)]
                segments.append(point)
        return segments

    def forward_kinematics(self, q):
        q0, q1, q2 = q[0], q[1], q[2]
        L1, L2, L34 = self.L1, self.L2, self.L34
        x = math.cos(q0) * (L2 * math.cos(q1) + L34 * math.cos(q1 + q2))
        y = math.sin(q0) * (L2 * math.cos(q1) + L34 * math.cos(q1 + q2))
        z = L1 + L2 * math.sin(q1) + L34 * math.sin(q1 + q2)
        return np.array([x, y, z])

    def jacobian(self, q):
        q0, q1, q2 = q[0], q[1], q[2]
        L2, L34 = self.L2, self.L34
        J = np.zeros((3, 5))

        r = L2 * math.cos(q1) + L34 * math.cos(q1 + q2)
        J[0, 0] = -math.sin(q0) * r
        J[1, 0] = math.cos(q0) * r
        J[2, 0] = 0.0

        J[0, 1] = -math.cos(q0) * (L2 * math.sin(q1) + L34 * math.sin(q1 + q2))
        J[1, 1] = -math.sin(q0) * (L2 * math.sin(q1) + L34 * math.sin(q1 + q2))
        J[2, 1] = L2 * math.cos(q1) + L34 * math.cos(q1 + q2)

        J[0, 2] = -math.cos(q0) * (L34 * math.sin(q1 + q2))
        J[1, 2] = -math.sin(q0) * (L34 * math.sin(q1 + q2))
        J[2, 2] = L34 * math.cos(q1 + q2)

        return J

    def update(self):
        if self.segment_index >= len(self.path_segments):
            self.segment_index = 0

        target_pos = np.array(self.path_segments[self.segment_index])
        current_q = self.current_joint_angles[:5]
        ee_pos = self.forward_kinematics(current_q)
        error = target_pos - ee_pos

        J = self.jacobian(current_q)
        JT = J[:, :3].T
        delta_q = JT @ error * 0.005

        desired_q = [current_q[i] + delta_q[i] if i < 3 else current_q[i] for i in range(5)]

        commanded_angles = []
        for i in range(5):
            pid_out = self.pid_controllers[i].update(desired_q[i], current_q[i], self.timer_period)
            final_angle = current_q[i] + pid_out
            commanded_angles.append(final_angle)

        self.publish_angles(commanded_angles)

        self.get_logger().info(
            f"[{self.segment_index}/{len(self.path_segments)}] Pos {target_pos} -> " +
            ", ".join(f"{math.degrees(a):.1f}°" for a in commanded_angles),
            throttle_duration_sec=1.0
        )

        self.segment_index += 1

    def publish_angles(self, angles):
        for i, angle in enumerate(angles):
            msg = Float64()
            msg.data = float(angle)
            self.joint_pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CartesianJacobianPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()