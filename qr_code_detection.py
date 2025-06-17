#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # Change this if your image topic is different
            self.image_callback,
            10
        )
        self.qr_pub = self.create_publisher(String, '/qr_code_data', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        decoded_objs = decode(frame)

        for obj in decoded_objs:
            qr_data = obj.data.decode('utf-8')
            self.get_logger().info(f"QR Code Detected: {qr_data}")
            self.qr_pub.publish(String(data=qr_data))
            # Draw a rectangle and text on the frame
            points = obj.polygon
            if len(points) > 1:
                pts = [(point.x, point.y) for point in points]
                cv2.polylines(frame, [np.array(pts, np.int32)], isClosed=True, color=(0,255,0), thickness=2)
            cv2.putText(frame, qr_data, (obj.rect.left, obj.rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            break  # Only process the first detected QR code

        # Show the image
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)  # Needed to refresh the OpenCV window

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
