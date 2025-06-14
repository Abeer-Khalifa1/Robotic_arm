import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class ManualJointController(Node):
    def __init__(self):
        super().__init__('manual_joint_controller')

        # Create publishers for joints 0–4
        self.joint_pubs = [
            self.create_publisher(Float64, f'/joint_{i}/position_cmd', 10)
            for i in range(5)
        ]

        # ✅ True vertical pose for your hardware
        self.stand_pose = [
            0.0,           # joint_0: base
            math.pi / 2,   # joint_1: shoulder upright (90°)
            math.pi / 2,   # joint_2: elbow straight up
            0.0,           # joint_3
            0.0            # joint_4
        ]

        self.start_time = time.time()
        self.get_logger().info("Starting 20s vertical stand...")

    def run_forever(self):
        warmup_time = 20.0        # Stand time before motion
        joint_duration = 3.0      # Each joint moves for 3 seconds
        rate_hz = 20
        period = 1.0 / rate_hz

        while rclpy.ok():
            t = time.time() - self.start_time

            if t < warmup_time:
                # Hold vertical posture
                for i, angle in enumerate(self.stand_pose):
                    msg = Float64()
                    msg.data = float(angle)
                    self.joint_pubs[i].publish(msg)

                self.get_logger().info(f"Holding vertical pose: {int(warmup_time - t)}s remaining...")
            else:
                # Animate one joint at a time
                motion_time = t - warmup_time
                joint_index = int(motion_time // joint_duration) % 5
                local_t = motion_time % joint_duration

                # Create new joint angles list
                angles = self.stand_pose[:]

                # Sine wave motion: ±0.5 rad around center
                if joint_index == 2:
                    center = math.pi / 2  # elbow vertical
                else:
                    center = self.stand_pose[joint_index]

                animated_angle = center + 0.5 * math.sin(2 * math.pi * 0.5 * local_t)
                animated_angle = max(0.0, min(math.pi, animated_angle))  # Clamp to [0, π]
                angles[joint_index] = animated_angle

                for i, angle in enumerate(angles):
                    msg = Float64()
                    msg.data = float(angle)
                    self.joint_pubs[i].publish(msg)

                self.get_logger().info(
                    f"Joint {joint_index} moving → " +
                    ", ".join(f"{a:.2f}" for a in angles)
                )

            time.sleep(period)

def main(args=None):
    rclpy.init(args=args)
    node = ManualJointController()
    try:
        node.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
