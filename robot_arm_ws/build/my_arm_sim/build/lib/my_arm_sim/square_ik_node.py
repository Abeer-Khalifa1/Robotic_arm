#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.joint_limits = {
            'joint_0': (0.0, 3.14),
            'joint_1': (0.0, 3.14),
            'joint_2': (0.0, 3.14),
            'joint_3': (0.0, 3.14),
            'joint_4': (0.0, 3.14)
        }
        
        self.joint_publishers = {
            joint: self.create_publisher(Float64, f'/model/Robot_arm/joint/{joint}/position_cmd', 10)
            for joint in self.joint_names
        }
        
        self.dh_params = {
            'base_height': 0.08,
            'a1': 0.015,
            'L1': 0.135,
            'L2': 0.16,
            'L3': 0.065
        }
        
        self.square_center = [0.3, 0.2]
        self.square_size = 0.1
        self.step_size = 0.02
        self.trajectory = self.generate_square_trajectory()
        self.current_target = self.trajectory[0]
        self.point_index = 0
        
        self.timer = self.create_timer(0.1, self.update_position)  # <- Changed here

    def generate_square_trajectory(self):
        half = self.square_size / 2
        return [
            [self.square_center[0] - half, self.square_center[1] - half],
            [self.square_center[0] + half, self.square_center[1] - half],
            [self.square_center[0] + half, self.square_center[1] + half],
            [self.square_center[0] - half, self.square_center[1] + half],
            [self.square_center[0] - half, self.square_center[1] - half]
        ]

    def inverse_kinematics(self, x, z):
        angles = {}
        try:
            x_adj = x - self.dh_params['a1']
            z_adj = z - self.dh_params['base_height']
            
            angles['joint_0'] = math.atan2(x_adj, z_adj)

            r = math.hypot(x_adj, z_adj)
            L1 = self.dh_params['L1']
            L2 = self.dh_params['L2']
            
            cos_theta2 = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
            cos_theta2 = max(-1.0, min(1.0, cos_theta2))
            theta2 = math.acos(cos_theta2)
            
            alpha = math.atan2(z_adj, x_adj)
            beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
            theta1 = alpha - beta
            
            theta3 = -theta1 - theta2
            theta4 = 0.0
            
            angles['joint_1'] = theta1
            angles['joint_2'] = theta2
            angles['joint_3'] = theta3
            angles['joint_4'] = theta4
            
            for joint in self.joint_names:
                angles[joint] = np.clip(
                    angles[joint],
                    self.joint_limits[joint][0],
                    self.joint_limits[joint][1]
                )
            
            self.get_logger().info(f"Raw angles (deg): {[round(math.degrees(ang), 2) for ang in angles.values()]}")
            return angles
            
        except Exception as e:
            self.get_logger().error(f"IK Error: {str(e)}")
            return None

    def update_position(self):
        target_point = self.trajectory[self.point_index]
        current_pos = self.current_target
        
        dx = target_point[0] - current_pos[0]
        dz = target_point[1] - current_pos[1]
        distance = math.hypot(dx, dz)
        
        if distance > self.step_size:
            ratio = self.step_size / distance
            new_x = current_pos[0] + dx * ratio
            new_z = current_pos[1] + dz * ratio
        else:
            new_x, new_z = target_point
            self.point_index = (self.point_index + 1) % len(self.trajectory)
        
        self.current_target = [new_x, new_z]
        
        angles = self.inverse_kinematics(new_x, new_z)
        
        if angles:
            self.publish_joints(angles)
            self.get_logger().info(f"Target X: {new_x:.3f}, Z: {new_z:.3f}")
        else:
            self.get_logger().warn("IK Solution not found!")

    def publish_joints(self, angles):
        for joint, angle in angles.items():
            msg = Float64()
            msg.data = angle
            self.joint_publishers[joint].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
############################################################
# ##Esp
# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
# from std_msgs.msg import Float64MultiArray
# from visualization_msgs.msg import Marker
# import math

# class SquareIKNode(Node):
#     def __init__(self):
#         super().__init__('square_ik_node')

#         # Declare parameters with descriptions
#         param_descriptors = {
#             'square_center_x': ParameterDescriptor(description='X position of square center (cm)'),
#             'square_center_z': ParameterDescriptor(description='Z position of square center (cm)'),
#             'square_size': ParameterDescriptor(description='Side length of square (cm)'),
#             'publish_topic': ParameterDescriptor(description='Joint angles topic name'),
#             'step_size': ParameterDescriptor(description='Interpolation step size (cm)', type=2)  # 2=double
#         }
        
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('square_center_x', 30.0, param_descriptors['square_center_x']),
#                 ('square_center_z', 20.0, param_descriptors['square_center_z']),
#                 ('square_size', 10.0, param_descriptors['square_size']),
#                 ('publish_topic', 'joint_angles', param_descriptors['publish_topic']),
#                 ('step_size', 2.0, param_descriptors['step_size'])
#             ]
#         )

#         # Setup publishers
#         self.joint_pub = self.create_publisher(Float64MultiArray, self.get_parameter('publish_topic').value, 10)
#         self.marker_pub = self.create_publisher(Marker, 'end_effector_marker', 10)

#         # Robot dimensions (cm)
#         self.a1 = 1.5   # Base X offset
#         self.d1 = 8.0   # Base Z height
#         self.a2 = 13.5  # Link 2 length
#         self.a3 = 16.0  # Link 3 length

#         # Trajectory control
#         self.square_points = self.generate_square()
#         self.point_index = 0
#         self.current_target = self.square_points[0]
#         self.step_size = self.get_parameter('step_size').value

#         # Create timer (2Hz update rate)
#         self.timer = self.create_timer(0.5, self.timer_callback)

#         # Parameter change callback
#         self.add_on_set_parameters_callback(self.parameter_changed)

#     def generate_square(self):
#         """Generate square path points with center and size from parameters"""
#         cx = self.get_parameter('square_center_x').value
#         cz = self.get_parameter('square_center_z').value
#         size = self.get_parameter('square_size').value
#         half = size / 2.0

#         return [
#             (cx - half, cz - half),  # A
#             (cx + half, cz - half),  # B
#             (cx + half, cz + half),  # C
#             (cx - half, cz + half),  # D
#             (cx - half, cz - half)   # A (close path)
#         ]

#     def parameter_changed(self, params):
#         """Handle parameter updates at runtime"""
#         needs_new_path = False
#         for param in params:
#             if param.name in ['square_center_x', 'square_center_z', 'square_size']:
#                 needs_new_path = True
#             elif param.name == 'step_size':
#                 self.step_size = param.value

#         if needs_new_path:
#             self.square_points = self.generate_square()
#             self.point_index = 0
#             self.current_target = self.square_points[0]

#         return SetParametersResult(successful=True)

#     def inverse_kinematics(self, x, z):
#         """Calculate joint angles for target position (returns None if unreachable)"""
#         try:
#             # Adjust for base offset
#             x_adj = x - self.a1
#             z_adj = z - self.d1
#             r = math.hypot(x_adj, z_adj)

#             # Calculate elbow angle
#             denominator = 2 * self.a2 * self.a3
#             if abs(denominator) < 1e-6:
#                 raise ValueError("Invalid arm dimensions")

#             cos_theta3 = (self.a2**2 + self.a3**2 - r**2) / denominator  # Critical fix here
#             if abs(cos_theta3) > 1:
#                 return None

#             theta3 = math.acos(cos_theta3)

#             # Calculate shoulder angle
#             k1 = self.a2 + self.a3 * math.cos(theta3)
#             k2 = self.a3 * math.sin(theta3)
#             theta2 = math.atan2(z_adj, x_adj) - math.atan2(k2, k1)

#             # Convert to degrees for limit checking
#             angles_deg = [
#                 math.degrees(0.0),       # Base rotation (fixed)
#                 math.degrees(theta2),
#                 math.degrees(theta3),
#                 0.0, 0.0  # Additional joints
#             ]

#             # Check joint limits (customize for your robot)
#             if any(abs(a) > 150 for a in angles_deg[:3]):
#                 raise ValueError("Joint limit exceeded")

#             return [0.0, theta2, theta3, 0.0, 0.0]

#         except Exception as e:
#             self.get_logger().warn(f"IK failed: {str(e)}")
#             return None

#     def publish_marker(self, x, z):
#         """Visualize target position in RViz"""
#         marker = Marker()
#         marker.header.frame_id = "base_link"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.scale.x = 1.0
#         marker.scale.y = 1.0
#         marker.scale.z = 1.0
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.pose.position.x = x / 100.0  # Convert cm to meters
#         marker.pose.position.z = z / 100.0
#         self.marker_pub.publish(marker)

#     def timer_callback(self):
#         """Main control loop with linear interpolation"""
#         # Get target and current positions
#         target_point = self.square_points[self.point_index]
#         current_x, current_z = self.current_target

#         # Calculate movement vector
#         dx = target_point[0] - current_x
#         dz = target_point[1] - current_z
#         distance = math.hypot(dx, dz)

#         if distance > self.step_size:
#             # Move towards target
#             ratio = self.step_size / distance
#             new_x = current_x + dx * ratio
#             new_z = current_z + dz * ratio
#         else:
#             # Reached target, move to next point
#             new_x, new_z = target_point
#             self.point_index = (self.point_index + 1) % len(self.square_points)

#         self.current_target = (new_x, new_z)

#         # Calculate and publish IK
#         angles = self.inverse_kinematics(new_x, new_z)
#         if angles:
#             msg = Float64MultiArray()
#             msg.data = angles
#             self.joint_pub.publish(msg)
#             self.publish_marker(new_x, new_z)
#             self.get_logger().info(
#                 f"Target: ({new_x:.1f}, {new_z:.1f}) | "
#                 f"Angles: [{math.degrees(angles[1]):.1f}°, {math.degrees(angles[2]):.1f}°]",
#                 throttle_duration_sec=1.0
#             )
#         else:
#             self.get_logger().warn("IK solution not found!", throttle_duration_sec=1.0)

# def main(args=None):
#     rclpy.init(args=args)
#     node = SquareIKNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()