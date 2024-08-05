import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import math
from controller import HandTrackingModule as htm

class TeleopTwistCV(Node):
    def __init__(self):
        super().__init__('teleop_cv')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        self.detector = htm.handDetector()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def timer_callback(self):
        success, img = self.cap.read()
        if success:
            # Flip the camera feed horizontally
            img = cv2.flip(img, 1)

            img = self.detector.findHands(img)
            lmlist = self.detector.findPosition(img)
            h, w, c = img.shape
            x_oren, y_oren = 'none', 'none'
            
            '''# Control partitions
            p = int(w / 3)
            cv2.line(img, (p, 0), (p, h), (0, 255, 0), 2)
            cv2.line(img, (2 * p, 0), (2 * p, h), (0, 255, 0), 2)'''

            if len(lmlist) != 0:
                cx, cy = lmlist[0][1], lmlist[0][2]
                x1, y1 = lmlist[4][1], lmlist[4][2]
                x2, y2 = lmlist[8][1], lmlist[8][2]
                cv2.circle(img, (x1, y1), 2, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 2, (255, 0, 255), cv2.FILLED)
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)

                l = int(math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2)))

                if cx > (2 * w / 3):
                    cv2.putText(img, 'Right', (cx + 10, cy + 10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 0), 1)
                    x_oren = 'Right'
                    self.angular_velocity = 0.25
                elif cx < (w / 3):
                    cv2.putText(img, 'Left', (cx + 10, cy + 10), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 1)
                    x_oren = 'Left'
                    self.angular_velocity = -0.25
                else:
                    self.angular_velocity = 0.0

                if l > 120:
                    cv2.putText(img, 'Forward', (cx + 10, cy + 50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1)
                    y_oren = 'Forward'
                    self.linear_velocity = 0.25
                elif l < 30:
                    cv2.putText(img, 'Backward', (cx + 10, cy + 50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 1)
                    y_oren = 'Backward'
                    self.linear_velocity = -0.25
                else:
                    self.linear_velocity = 0.0

            twist = Twist()
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity
            self.publisher_.publish(twist)

            cv2.imshow('Image', img)
            cv2.waitKey(1)

        else:
            self.get_logger().info('Failed to capture frame from camera')

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_cv = TeleopTwistCV()
    rclpy.spin(teleop_twist_cv)
    teleop_twist_cv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
