import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

class JointArraySplitter(Node):
    def __init__(self):
        super().__init__('joint_array_splitter')
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_angles',
            self.callback,
            10
        )
        self.pubs = [
            self.create_publisher(Float64, f'/joint_{i}/position_cmd', 10)
            for i in range(5)  # For 5 joints
        ]

    def callback(self, msg):
        if len(msg.data) != len(self.pubs):
            self.get_logger().error("Joint count mismatch!")
            return
        for pub, angle in zip(self.pubs, msg.data):
            pub_msg = Float64()
            pub_msg.data = angle
            pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointArraySplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()