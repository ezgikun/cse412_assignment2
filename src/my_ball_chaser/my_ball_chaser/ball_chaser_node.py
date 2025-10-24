#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class BallChaserNode(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        self.get_logger().info('Ball Chaser Node started (Edge Detection Version).')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.twist_msg = Twist()
        self.width = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        self.width = cv_image.shape[1]
        height = cv_image.shape[0]

        # Sadece görüntünün alt yarısına bak (ROI)
        start_row = int(height * 0.55)
        roi_image = cv_image[start_row:, :]

        # Görüntüyü gri tona çevir ve bulanıklaştır
        gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Kenarları bul (Canny Edge Detection)
        edges = cv2.Canny(blurred, 50, 150)

        # Kenarlardaki şekilleri (konturları) bul
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_ball = False
        if contours:
            # En büyük konturu (şekli) bul
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Eğer bu şekil yeterince büyükse (gürültü değilse)
            if cv2.contourArea(largest_contour) > 20:
                # Şeklin etrafına bir daire çiz ve merkezini bul
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                
                # Merkezi (cx) ve (cy) al
                cx = int(x)
                cy = int(y) + start_row # ROI'den tam görüntüye geri çevir
                
                # Orijinal görüntüye kırmızı daireyi çiz
                cv2.circle(cv_image, (cx, cy), int(radius), (0, 0, 255), 2)
                
                self.drive_robot(cx)
                found_ball = True

        if not found_ball:
            # Eğer top bulunamadıysa (kontur yoksa) DUR
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.publisher_.publish(self.twist_msg)

        # Görsel pencereler
        cv2.imshow("Ball Chaser View (Edges)", cv_image)
        cv2.imshow("Mask View (Edges)", edges)
        cv2.waitKey(1)

    def drive_robot(self, cx):
        left_bound = self.width / 3
        right_bound = self.width * 2 / 3

        if cx < left_bound:
            # Sola dön
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.3
        elif cx > right_bound:
            # Sağa dön
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = -0.3
        else:
            # Merkezde, ileri git
            self.twist_msg.linear.x = 0.2
            self.twist_msg.angular.z = 0.0
            
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallChaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
