#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class QrCodeNavigator(Node):
    def __init__(self):
        super().__init__('qr_code_navigator')
        # Subscribe to QR code data (assumed published on '/qr_code_data')
        self.subscription = self.create_subscription(
            String,
            '/qr_code_data',
            self.qr_callback,
            10
        )
        # Publisher for navigation goal (for example, on '/goal_pose')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Define some example location waypoints
        self.locations = {
            "entrance": (-0.22, -1.65),
            "sir_desk": (-6.60, -1.71),
            "dye_kit_feeder_station": (-2.95, 9.60),
            "jenatics_automation_kit": (-0.98, 5.13),
            "first_computer_table_line": (-3.34, 1.14),
            "second_computer_table_line": (-3.25, 3.04),
            "last_computer_table_line": (-5.18, 7.77),
            "charging_station": (-7.07, 9.76),
            "gpt_computer": (-3.36, 7.59),
            "home": (-4.27, 7.55)
        }

    def qr_callback(self, msg):
        location = msg.data.strip()
        if location in self.locations:
            x, y = self.locations[location]
            self.get_logger().info(f"QR Code for '{location}' detected. Navigating to ({x}, {y})")
            self.send_goal(x, y)
        else:
            self.get_logger().warn(f"Unknown location from QR Code: '{location}'")

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # Facing forward
        self.goal_pub.publish(goal)
        self.get_logger().info("Navigation goal published.")

def main(args=None):
    rclpy.init(args=args)
    node = QrCodeNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
